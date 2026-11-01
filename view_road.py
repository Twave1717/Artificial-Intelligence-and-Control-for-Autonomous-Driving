"""팔달문 도로를 학습 없이 확인한다.

    python view_road.py
    python view_road.py --scenario busy --nogui --seconds 600
    python view_road.py --netedit

지도 원본: maps/paldalmun/source/ (설정·경로: map.json)
생성 결과: results/preview_paldalmun_<시각>/sumo/

NetEdit는 생성 결과를 열기 때문에 원본을 덮어쓰지 않는다. 영구적인 변경은
scripts/build_road.py로 다시 만들고 검수한다.
"""
import argparse
import os
import subprocess

from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION
from env import road_builder
from env.experiment import add_run_arguments, configure_execution, new_output_dir, write_run_config
# sumo_env를 import하면 SUMO_HOME 자동 감지 로직이 함께 실행된다
from env.sumo_env import SumoHighwayEnv  # noqa: F401  (자동 감지 목적의 import)
from sumolib import checkBinary

BASE = os.path.dirname(os.path.abspath(__file__))


def print_summary():
    """현재 road_config 내용을 표로 요약 출력."""
    print("─" * 46)
    print(" 선택한 지도와 교통 조건")
    print("─" * 46)
    print(f"  지도 / 조건    : {ROAD['map_id']} / {ROAD['scenario']}")
    print(f"  원본 상태      : {ROAD['network']['source_status']} / {ROAD['map_version']}")
    print(f"  ego 경로       : {ROAD['ego_route']} ({ROAD['length']} m)")
    print(f"  출발 구간 차선 : {ROAD['num_lanes']}")
    print(f"  제한속도       : {ROAD['speed_limit']} m/s "
          f"(≈ {float(ROAD['speed_limit']) * 3.6:.0f} km/h)")
    print(f"  배경 교통량    : {TRAFFIC['vehs_per_hour']} 대/시간")
    print(f"  배경차 최고속도: {TRAFFIC['max_speed']} m/s")
    print(f"  ego 가속/감속  : +{EGO['accel']} / -{EGO['decel']} m/s²")
    print(f"  정류장 시설    : {len(ROAD['network'].get('bus_stops', []))}곳 (표시만, 버스 운행 없음)")
    print("─" * 46)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="도로 구조를 GUI로 확인")
    add_run_arguments(parser, episode_limit=False)
    parser.add_argument("--netedit", action="store_true",
                        help="sumo-gui 대신 netedit(도로 편집기)로 열기")
    parser.add_argument("--nogui", action="store_true",
                        help="화면 없이 SUMO가 설정을 읽고 끝까지 실행하는지만 확인 (GUI 없는 환경용)")
    parser.add_argument("--seconds", type=int, default=120,
                        help="--nogui 실행 시 시뮬레이션 길이(초). 기본 120초. "
                             "(GUI 모드는 시간 제한 없이 사용자가 직접 제어)")
    args = parser.parse_args()
    if args.seconds < 1 or (args.netedit and args.nogui):
        parser.error("seconds는 양수이며 netedit/nogui는 동시에 사용할 수 없습니다.")
    try:
        configure_execution(args)
    except ValueError as exc:
        parser.error(str(exc))
    run_dir = new_output_dir(os.path.join(BASE, "results"), "preview")
    write_run_config(run_dir, kind="preview")

    # 1) 선택한 지도만 실행 폴더에 생성한다. 원본이나 다른 실행에는 영향이 없다.
    sumocfg = road_builder.build(ROAD, EGO, TRAFFIC, os.path.join(run_dir, "sumo"))
    net_file = os.path.join(run_dir, "sumo", "network.net.xml")
    print(f"도로 생성 완료: {sumocfg}")
    print_summary()

    # 2) 요청한 모드로 열기
    if args.netedit:
        # netedit: SUMO에 포함된 도로망 편집기. 노드/엣지/차선을 시각적으로 확인.
        print("netedit 실행 중... (창을 닫으면 종료)")
        subprocess.run([checkBinary("netedit"), net_file])

    elif args.nogui:
        # 화면 없이 짧게 돌려 SUMO가 생성한 설정을 읽고 끝까지 실행하는지만 본다.
        result = subprocess.run(
            [checkBinary("sumo"), "-c", sumocfg, "--end", str(args.seconds),
             "--seed", str(args.seed), "--no-step-log", "true",
             "--duration-log.statistics", "true"],   # 종료 시 차량 통계 출력
            capture_output=True, text=True)
        # SUMO의 통계 요약(투입/도착 차량 수 등)을 그대로 보여준다
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.stderr.strip():
            print(result.stderr.strip())
        if result.returncode == 0:
            print(f"SUMO가 생성한 설정을 읽고 {args.seconds}초를 오류 없이 실행했습니다. "
                  "충돌·순간이동·도착 검사는 scripts/validate_road.py가 합니다.")
        else:
            print(f"SUMO 실행 실패 (종료 코드 {result.returncode})")
        raise SystemExit(result.returncode)

    else:
        # sumo-gui: 도로를 "정적으로" 띄운다.
        #   --start, --quit-on-end, --end 를 모두 주지 않는 것이 핵심:
        #     --start 없음      → 자동 재생 안 함. 창이 뜨면 도로만 보이고,
        #                         사용자가 ▶(플레이) 버튼을 눌러야 차가 움직인다.
        #     --quit-on-end 없음 → 시뮬레이션이 끝나도 창이 저절로 닫히지 않는다.
        #     --end 없음        → 시간 제한 없음. 원하는 만큼 돌려볼 수 있다.
        #   --delay 100 : 재생 시 스텝당 100ms (0이면 눈으로 못 따라갈 만큼 빠름).
        #                 GUI 상단 Delay 입력칸에서 실행 중에도 바꿀 수 있다.
        print("sumo-gui 실행 중...")
        print("  · 창이 뜨면 도로만 보입니다. ▶(플레이) 버튼을 눌러야 차가 움직입니다.")
        print(f"  · 확대: env/mdp_config.py의 gui_zoom={SIMULATION['gui_zoom']}")
        print("  · 속도 조절: 상단 Delay(ms) 값을 조정 (크게 = 느리게)")
        print("  · 종료하려면 창을 닫으세요.")
        subprocess.run([checkBinary("sumo-gui"), "-c", sumocfg, "--seed", str(args.seed)])
