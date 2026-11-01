"""실행 진입점(train/test/collect/view)이 공유하는 선택·기록 도우미.

도로는 팔달문 하나다. 진입점은 교통 조건(--scenario/--seed)과 ego 경로(--ego-route)를
같은 이름으로 받고, 에피소드를 돌리는 train/test/collect는 에피소드 길이(--max-steps)도 받는다.
결과 폴더에는 지도 버전·도로망 해시·적용한 설정·코드 커밋을 run_config.json으로 남긴다.
"""
from copy import deepcopy
from datetime import datetime
from importlib import metadata
import json
from pathlib import Path
import subprocess
import sys

from env.road_config import ROAD, EGO, TRAFFIC, SCENARIOS, configure_map
from env.mdp_config import SIMULATION, OBSERVATION, ACTION

BASE = Path(__file__).resolve().parents[1]


def add_run_arguments(parser, episode_limit=True):
    parser.add_argument("--scenario", choices=tuple(SCENARIOS), default="baseline",
                        help="교통량: quiet(0.5배), baseline(기본 수요), busy(1.25배)")
    parser.add_argument("--seed", type=int, default=0, help="환경 난수 시드")
    parser.add_argument("--ego-route", default=None,
                        help="ego가 달릴 map.json 경로 id; 생략하면 map.json의 ego_route(r0)")
    if episode_limit:
        parser.add_argument("--max-steps", type=int, default=None,
                            help="에피소드 시간 제한(RL 스텝); 생략하면 mdp_config 기본값")


def configure_execution(args):
    """팔달문 도로와 교통 조건을 적용한다. ValueError는 호출자가 CLI 오류로 바꾼다."""
    configure_map(args.scenario, args.seed, args.ego_route)
    max_steps = getattr(args, "max_steps", None)
    if max_steps is not None:
        if max_steps < 1:
            raise ValueError("--max-steps는 1 이상이어야 합니다.")
        SIMULATION["max_steps"] = max_steps


def new_output_dir(root, kind):
    """같은 종류를 동시에 실행해도 서로의 SUMO 파일을 덮어쓰지 않는 폴더."""
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    path = Path(root) / f"{kind}_{ROAD['map_id']}_{stamp}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def _git_state():
    """HEAD 커밋과, 커밋과 다른 파일이 있었는지. git describe --dirty와 달리
    추적하지 않는 새 파일도 dirty로 센다. 그 파일로 돌린 결과는 커밋만으로 재현되지 않는다."""
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=BASE, text=True,
                                       stderr=subprocess.DEVNULL).strip()
    try:
        return {"commit": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        return None


def write_run_config(directory, *, hparams=None, reward=None, settings=None, **extra):
    """적용한 설정과 도로 버전을 JSON으로 남긴다. 모델·데이터의 출처 추적용이다.

    settings를 주면 지금 설정 대신 그 road/ego/traffic/simulation/observation/action을
    기록한다. BC 모델은 데이터셋을 모은 설정에서 나왔으므로 데이터셋 metadata의 값을 넘긴다.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    versions = {"python": sys.version.split()[0]}
    for package in ("numpy", "gymnasium", "torch", "sumolib", "traci"):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            versions[package] = None
    if settings is None:
        # 지도 전체(native XML 포함)는 크다. 버전과 해시만 남긴다.
        road = {key: value for key, value in ROAD.items() if key != "network"}
        road["network_sha256"] = ROAD["network"]["network_source"]["sha256"]
        settings = {"road": road, "ego": EGO, "traffic": TRAFFIC, "simulation": SIMULATION,
                    "observation": OBSERVATION, "action": ACTION}
    payload = {
        **deepcopy(settings), "reward": deepcopy(reward), "hparams": deepcopy(hparams),
        "git": _git_state(), "versions": versions,
        "created_at": datetime.now().astimezone().isoformat(),
        **extra,
    }
    (directory / "run_config.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return payload


def read_run_config(model_path):
    """모델 옆의 run_config.json을 읽는다. 없으면 None."""
    path = Path(model_path).resolve().parent / "run_config.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))
