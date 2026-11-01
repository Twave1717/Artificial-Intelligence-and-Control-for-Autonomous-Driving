"""학습된 모델을 sumo-gui로 재생하고 주행 지표를 출력한다.

사용법:
    python test.py results/run_20260824_153012/model.pt   ← 모델 경로 지정
    python test.py                                        ← run_config.json이 있는 최근 run 자동 선택
    python test.py --episodes 5 --nogui                   ← 화면 없이 지표만 측정
    python test.py results/bc_example/model.pt --algorithm bc --nogui
    python test.py MODEL --episodes 1 --overview  ← 지도 전체 초기 화면 유지

--scenario, --ego-route, --max-steps를 생략하면 run_config.json에 적힌 학습 당시 값으로 평가한다.
학습 때와 다른 설정은 평가 전에 모두 출력한다. 결정 간격(decision_interval)이 다르면 평가하지 않는다.

빨간 차가 ego. 앞차가 느리면 감속하고 앞이 비면 가속하는지 관찰해보자.
"""
import argparse
import glob
import json
import os
import torch

from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env import road_builder
from env.sumo_env import SumoHighwayEnv
from env.experiment import (add_run_arguments, configure_execution, new_output_dir,
                            read_run_config, write_run_config)
from maps import load_map

from algorithms.ppo import PPO
from algorithms.bc import BCPolicy
from utils.evaluator import evaluate_policy

# train.py와 동일한 네트워크 구조로 만들어야 가중치를 불러올 수 있으므로
# 기본값은 train.py와 공유하고, 모델 옆 run_config.json의 저장값으로 복원한다.
from train import HPARAMS, RESULTS_DIR, REWARD

BASE = os.path.dirname(os.path.abspath(__file__))


def find_latest_model():
    """run_config.json이 있는 results/*/model.pt 중 가장 최근에 저장된 것. 없으면 None.

    run_config.json이 없는 모델(예: 수업 원본 고속도로 모델)은 어느 도로에서
    학습했는지 알 수 없으므로 고르지 않는다.
    """
    candidates = [path for path in glob.glob(os.path.join(BASE, RESULTS_DIR, "*", "model.pt"))
                  if os.path.exists(os.path.join(os.path.dirname(path), "run_config.json"))]
    return max(candidates, key=os.path.getmtime) if candidates else None


def load_agent(model_path, obs_dim, act_dim, algorithm="auto"):
    """BC 체크포인트와 기존 PPO state_dict를 구분해 평가 정책을 복원한다."""
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    if not isinstance(checkpoint, dict):
        raise ValueError("지원하지 않는 모델 파일 형식입니다.")
    if checkpoint.get("algorithm") == "bc":
        detected = "bc"
    elif "trunk.0.weight" in checkpoint and "mu_head.weight" in checkpoint:
        detected = "ppo"
    else:
        raise ValueError("현재 BC 체크포인트 또는 PPO state_dict가 아닙니다.")
    if algorithm != "auto" and algorithm != detected:
        raise ValueError(f"--algorithm {algorithm}과 모델 유형 {detected}가 다릅니다.")

    if detected == "bc":
        if not {"state_dim", "action_dim", "state_dict"}.issubset(checkpoint):
            raise ValueError("현재 BCPolicy 형식이 아닙니다. 현재 train_bc.py로 학습한 모델을 사용하세요.")
        dims = (checkpoint["state_dim"], checkpoint["action_dim"])
        if dims != (obs_dim, act_dim):
            raise ValueError(f"BC 모델 입출력 차원 {dims} != 환경 {(obs_dim, act_dim)}")
        agent = BCPolicy.load(model_path, device="cpu")
    else:
        # 다른 컴퓨터에서 학습한 CUDA 모델도 CPU로 재생할 수 있다.
        agent = PPO(obs_dim=obs_dim, act_dim=act_dim, **{**HPARAMS, "device": "cpu"})
        try:
            agent.policy.load_state_dict(checkpoint)
        except RuntimeError as exc:
            raise ValueError("PPO 모델과 현재 환경 차원 또는 HPARAMS의 네트워크 구조가 다릅니다.") from exc
        agent.policy.eval()
    print(f"평가 알고리즘: {detected.upper()}")
    return agent


def comparable_settings(config, scenario):
    """run_config.json과 같은 모양의 JSON 값. GUI 표시 설정은 주행 결과와 무관해 뺀다."""
    return json.loads(json.dumps({
        "scenario": scenario, **{block: config[block] for block in ("ego", "traffic", "observation", "action")},
        "simulation": {key: value for key, value in config["simulation"].items() if not key.startswith("gui_")}}))


def setting_differences(then, now, prefix=""):
    """두 설정에서 값이 다른 키를 (점으로 이은 이름, 학습 값, 평가 값)으로 모은다."""
    rows = []
    for key in sorted(then.keys() | now.keys()):
        old, new = then.get(key), now.get(key)
        if isinstance(old, dict) and isinstance(new, dict):
            rows += setting_differences(old, new, f"{prefix}{key}.")
        elif old != new:
            rows.append((prefix + key, old, new))
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="학습된 모델 재생/평가")
    add_run_arguments(parser)
    # 생략한 교통 조건은 학습 run의 값으로 채운다.
    parser.set_defaults(scenario=None)
    parser.add_argument("model", nargs="?", default=None,
                        help="model.pt 경로 (생략 시 run_config.json이 있는 최근 run 자동 선택)")
    parser.add_argument("--episodes", type=int, default=3,
                        help="재생/평가할 에피소드 수")
    parser.add_argument("--nogui", action="store_true",
                        help="화면 없이 주행 지표만 측정 (빠름)")
    parser.add_argument("--overview", action="store_true",
                        help="ego 추적 대신 지도 전체 초기 화면 유지")
    parser.add_argument("--algorithm", choices=["auto", "ppo", "bc"], default="auto",
                        help="기본은 모델 파일에서 PPO/BC 자동 판별")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes는 1 이상이어야 합니다.")

    model_path = args.model or find_latest_model()
    if model_path is None:
        parser.error(f"'{RESULTS_DIR}/' 안에 run_config.json이 있는 model.pt가 없습니다. "
                     "먼저 train.py 또는 train_bc.py로 학습하세요.")
    if not os.path.isabs(model_path):
        model_path = os.path.join(BASE, model_path)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"모델 파일이 없습니다: {model_path}")
    print(f"모델 로드: {model_path}")

    # 어느 도로·설정에서 학습했는지 모르는 모델은 평가하지 않는다.
    saved = read_run_config(model_path)
    if saved is None:
        parser.error(f"{os.path.join(os.path.dirname(model_path), 'run_config.json')}이 없습니다. "
                     "이 도로에서 train.py 또는 train_bc.py로 학습한 run만 평가할 수 있습니다.")
    trained, current = saved["road"], load_map()
    if ((trained["map_id"], trained["map_version"], trained.get("network_sha256"))
            != (current["id"], current["version"], current["network_source"]["sha256"])):
        parser.error(f"모델은 지도 {trained['map_id']} {trained['map_version']}에서 학습됐고 "
                     f"현재 도로는 {current['id']} {current['version']}입니다.")
    # 생략한 ego 경로·교통 조건·에피소드 길이는 학습 당시 값으로 평가한다.
    simulation = saved["simulation"]
    if args.ego_route is None:
        args.ego_route = trained["ego_route"]
    if args.scenario is None:
        args.scenario = trained["scenario"]
    if args.max_steps is None:
        args.max_steps = simulation["max_steps"]
    try:
        configure_execution(args)
    except ValueError as exc:
        parser.error(str(exc))
    # 결정 간격을 바꾸는 옵션은 없다. 다르면 gamma와 스텝당 보상이 다른 시간을 뜻한다.
    # decision_interval이 없는 run은 SUMO 한 스텝(step_length)이 RL 한 스텝이었다.
    trained_interval = simulation.get("decision_interval", simulation["step_length"])
    if trained_interval != SIMULATION["decision_interval"]:
        parser.error(f"decision_interval: 학습 {trained_interval}초, 현재 {SIMULATION['decision_interval']}초. "
                     "이 값은 명령줄에서 바꿀 수 없습니다.")
    # PPO 가중치를 불러오려면 학습 당시의 네트워크 구조(hidden_sizes, policy_type)가 필요하고,
    # 보상 합을 학습 곡선과 비교하려면 학습 당시의 보상 계수가 필요하다.
    # BC run의 hparams는 BC 학습 설정이라 PPO 구조에 섞지 않는다.
    if saved["kind"] != "bc_training":
        HPARAMS.update(saved.get("hparams") or {})
    code_reward = dict(REWARD)
    REWARD.clear()
    REWARD.update(saved["reward"])
    changed = sorted(key for key in code_reward.keys() | REWARD.keys()
                     if code_reward.get(key) != REWARD.get(key))
    if changed:
        print(f"※ 보상은 학습 당시 값으로 평가합니다. 현재 train.py와 다른 항목: {', '.join(changed)}")
    # 명령줄로 바꾼 값과 학습 뒤 코드에서 바뀐 설정(예: ego.speed_mode)을 거절하지 않고 모두 보여 준다.
    differences = setting_differences(
        comparable_settings(saved, trained["scenario"]),
        comparable_settings({"ego": EGO, "traffic": TRAFFIC, "observation": OBSERVATION, "action": ACTION,
                             "simulation": SIMULATION}, ROAD["scenario"]))
    if differences:
        print("※ 학습 당시와 다른 설정으로 평가합니다:")
        for key, then, now in differences:
            print(f"   {key}: 학습 {then} → 평가 {now}")
    run_dir = new_output_dir(os.path.join(BASE, "results"), "evaluation")
    write_run_config(run_dir, hparams=saved.get("hparams"), reward=REWARD,
                     kind="evaluation", source_model=os.path.abspath(model_path))
    sumocfg = road_builder.build(ROAD, EGO, TRAFFIC,
                                 os.path.join(run_dir, "sumo"))
    env = SumoHighwayEnv(
        cfg_path=sumocfg,
        road=ROAD, ego=EGO,
        mdp_sim={**SIMULATION, "gui_track_ego": False} if args.overview else SIMULATION,
        mdp_obs=OBSERVATION,
        action=ACTION, reward=REWARD,
        gui=not args.nogui,
        gui_autostart=False,   # ▶(플레이) 버튼을 눌러야 에피소드 시작
    )

    if not args.nogui:
        print("※ 창이 뜨면 ▶(플레이) 버튼을 눌러야 에피소드가 시작됩니다.")
        print("  에피소드가 끝나면 다음 에피소드 창이 다시 뜨고, 또 ▶를 누르면 됩니다.")

    try:
        agent = load_agent(model_path, env.observation_space.shape[0],
                           env.action_space.shape[0], args.algorithm)
        metrics = evaluate_policy(agent, env, n_episodes=args.episodes,
                                  deterministic=True, seed=args.seed)
    finally:
        env.close()

    with open(os.path.join(run_dir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump({"model": os.path.abspath(model_path), "scenario": args.scenario,
                   "episode_seeds": list(range(args.seed, args.seed + args.episodes)),
                   "metrics": metrics.as_dict()}, f, ensure_ascii=False, indent=2)
    print(f"평가 결과: {run_dir}")

    print(f"\n─── 주행 지표 ({args.episodes} 에피소드 평균) ───")
    print(f"  충돌률       : {metrics.collision_rate:.0%}")
    print(f"  완주율       : {metrics.success_rate:.0%}")
    print(f"  시간초과율   : {metrics.timeout_rate:.0%}")
    print(f"  오류 종료율  : {metrics.error_rate:.0%}")
    print(f"  전체 충돌률  : {metrics.simulation_collision_rate:.0%} (배경차 포함)")
    print(f"  속도 개입률  : {metrics.speed_intervention_rate:.0%} (SUMO 보조 포함)")
    print(f"  평균 속도    : {metrics.mean_speed:.2f} m/s "
          f"({metrics.mean_speed * 3.6:.1f} km/h)")
    print(f"  평균 차간거리: {metrics.mean_gap:.2f} m")
    print(f"  최소 차간거리: {metrics.min_gap:.2f} m")
    print(f"  차선변경     : {metrics.lane_changes:.1f} 회/에피소드")
    print(f"  평균 보상    : {metrics.ep_return:.2f}")
    print(f"  평균 길이    : {metrics.ep_length:.0f} 스텝 "
          f"({metrics.ep_length * SIMULATION['decision_interval']:.1f}초)")
