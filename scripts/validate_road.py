"""팔달문 도로의 연결과 차량 흐름을 같은 조건에서 확인하고 한 파일에 적는다.

    python scripts/validate_road.py --seeds 0 7 42 --scenarios quiet baseline busy --seconds 1800 \\
        --out maps/paldalmun/references/stability_validation.json

먼저 validate_structure로 경로의 차선 연결·정류장·ego 경로·설계 구역을 실행망에 대조한다.
그다음 seed×시나리오마다 두 종류를 프로세스 풀에서 돌린다.
- 배경: 배경 차량만 --seconds 동안. 충돌·순간이동·급제동·평균 주행 대수·최대 대기와,
  t=300초부터의 시나리오 차로(북쪽 합류 37394478#4, 남쪽 합류 -37394478#5, 좌회전 접근 -37394478#0)
  평균 속도·점유율·대기 시간.
- ego: r0와 모든 s_* 경로를 경로 추종 규칙으로 한 에피소드씩. 투입 대기·종료 사유·에피소드 길이·
  최장 정지·보상 합.
통과 기준(PASS_CONDITION): 배경은 충돌 0, 순간이동 0(300초 기준), 모든 경계 진입로에서 출발한 차량의 도착.
ego는 충돌·투입 실패 없음. r0는 모든 실행에서 도착. s_*는 도착, 시간 초과,
300초 정지 뒤 순간이동을 시나리오 결과로 기록만 한다. 좌회전·합류 실패가 곧 그 시나리오이기
때문이다. 이 검사는 도로와 파이프라인의 정상 동작을 확인한다. 학습 정책의 안전성이나 실제 도로와의
일치도를 입증하는 시험은 별도로 설계해야 한다.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from sumolib import checkBinary
from sumolib.net import readNet

from env import road_builder
from env.route_policy import route_following_action
from env.road_config import ROAD, EGO, TRAFFIC, configure_map
from env.mdp_config import SIMULATION

SCENARIO_LANES = ["37394478#4_0", "37394478#4_1", "-37394478#5_0", "-37394478#5_1", "-37394478#0_0", "-37394478#0_1"]
LANE_DATA_BEGIN_S = 300
PASS_CONDITION = ("배경: 충돌 0, 순간이동 0(300초 기준), 모든 경계 진입로에서 도착. "
                  "ego: 충돌·투입 실패 없음. r0는 모든 실행에서 도착. s_*는 도착, 시간 초과, 300초 정지 뒤 "
                  "순간이동을 시나리오 결과로 기록만 한다.")
# s_* 경로는 좌회전·합류 실패가 시나리오의 일부다. 규칙 운전자가 막혀 서 있는 것은 도로 결함이 아니다.
SCENARIO_OUTCOMES = ("arrived", "time_limit", "teleport")
# 설계 구역(map.json design.zones)에서 id를 담는 키와 그 종류. 원형 차도 진입로는 note 글에만 있다.
ZONE_ID_KEYS = {"edges": "edge", "alley": "edge", "joins": "edge", "approach": "edge", "left_to": "edge",
                "goal_edge": "edge", "node": "node", "route": "route"}
NOTE_EDGE_ID = re.compile(r"-?\d{6,}(?:#\d+)?")


def zone_ids(value, key=None):
    """설계 구역이 가리키는 (종류, id)를 모두 낸다. 종류는 edge, node, route다."""
    if isinstance(value, dict):
        for child_key, child in value.items():
            yield from zone_ids(child, child_key)
    elif isinstance(value, list):
        for child in value:
            yield from zone_ids(child, key)
    elif key == "note":
        yield from (("edge", edge_id) for edge_id in NOTE_EDGE_ID.findall(value))
    elif key in ZONE_ID_KEYS:
        yield ZONE_ID_KEYS[key], value


def validate_structure(definition: dict, net_path: Path) -> dict:
    """경로의 실제 차선 연결, 정류장 위치, ego 경로, 설계 구역이 가리키는 id를 실행망에 대조한다."""
    net = readNet(str(net_path), withInternal=True)
    errors = []
    checked_connections = 0
    for route_id, edge_ids in definition["routes"].items():
        edges = [net.getEdge(edge_id) for edge_id in edge_ids]
        for left, right in zip(edges, edges[1:]):
            if not left.getConnections(right):
                errors.append(f"{route_id}: {left.getID()} → {right.getID()}에 차선 연결이 없습니다.")
            checked_connections += 1
    # 정류장은 시설로만 표시하지만 위치가 실제 차선 안에 있어야 sumo-gui가 그린다.
    for stop in definition.get("bus_stops", []):
        try:
            lane = net.getLane(stop["lane"])
        except KeyError:
            errors.append(f"정류장 {stop['id']}: 차선 {stop['lane']}이 없습니다.")
            continue
        start, end = float(stop["startPos"]), float(stop["endPos"])
        if not 0 <= start < end <= lane.getLength():
            errors.append(f"정류장 {stop['id']}: {start}~{end}m가 "
                          f"차선 길이 {lane.getLength():.2f}m를 벗어납니다.")
    # 학습 진입점과 다른 담당자는 이 이름들로 도로를 찾는다.
    if definition["ego_route"] not in definition["routes"]:
        errors.append(f"ego 경로 {definition['ego_route']}: 경로 목록에 없습니다.")
    found = {"edge": net.hasEdge, "node": net.hasNode, "route": lambda name: name in definition["routes"]}
    for kind, name in zone_ids(definition["design"]["zones"]):
        if not found[kind](name):
            errors.append(f"설계 구역의 {kind} {name}: {'경로 목록' if kind == 'route' else '실행망'}에 없습니다.")
    if errors:
        raise ValueError("\n".join(errors))
    return {"edges": len(net.getEdges()), "routes": len(definition["routes"]),
            "checked_connections": checked_connections,
            "bus_stops": len(definition.get("bus_stops", []))}


def _number(element: ET.Element | None, key: str) -> int:
    return int(float(element.get(key, "0"))) if element is not None else 0


def run_background(folder: Path, scenario: str, seed: int, seconds: float) -> dict:
    """배경 차량만 실행해 충돌·순간이동·경계 진입로별 도착을 잰다."""
    configure_map(scenario, seed)
    definition = ROAD["network"]
    cfg = Path(road_builder.build(ROAD, EGO, TRAFFIC, str(folder)))
    (folder / "measure.add.xml").write_text(
        f'<additional><laneData id="scenario_lanes" file="lanes.xml" begin="{LANE_DATA_BEGIN_S}"/></additional>\n',
        encoding="utf-8")
    command = [
        checkBinary("sumo"), "-c", str(cfg), "--additional-files", "map.add.xml,measure.add.xml",
        "--begin", "0", "--end", str(seconds), "--time-to-teleport", "300",
        "--no-step-log", "true", "--duration-log.disable", "true",
        "--summary-output", "summary.xml", "--tripinfo-output", "tripinfo.xml",
        "--statistic-output", "statistics.xml", "--collision-output", "collisions.xml",
    ]
    completed = subprocess.run(command, cwd=folder, capture_output=True, text=True, timeout=900)
    (folder / "sumo.stderr.log").write_text(completed.stderr, encoding="utf-8")
    if completed.returncode:
        raise RuntimeError(f"SUMO 종료 코드 {completed.returncode}: {completed.stderr[-3000:]}")
    errors = re.findall(r"^Error:.*$", completed.stderr, flags=re.MULTILINE)
    if errors:
        raise RuntimeError("\n".join(errors))
    summary = ET.parse(folder / "summary.xml").getroot().findall("step")
    if not summary or float(summary[-1].get("time", "0")) < seconds - SIMULATION["step_length"] - 1e-6:
        raise ValueError("요청한 시뮬레이션 시간을 끝까지 실행하지 못했습니다.")
    warmup = SIMULATION["warmup_steps"] * SIMULATION["step_length"]
    window = [row for row in summary if float(row.get("time")) >= warmup]
    statistics = ET.parse(folder / "statistics.xml").getroot()
    teleports = statistics.find("teleports")
    teleport_count = _number(teleports, "total") if teleports is not None and "total" in teleports.attrib else 0
    # SUMO 버전에 따라 통계 필드가 달라도 경고에서 순간이동을 놓치지 않는다.
    teleport_count = max(teleport_count, len(re.findall(r"Teleporting vehicle", completed.stderr)))
    trips = ET.parse(folder / "tripinfo.xml").getroot().findall("tripinfo")
    arrived = [trip for trip in trips if float(trip.get("arrival", "-1")) >= 0]
    # 출발·도착 쌍 하나는 시간당 몇 대뿐이라 1,800초 안에 한 대도 없을 수 있다. 진입로 단위로 본다.
    entry_of = {flow["id"]: flow["from"] for flow in definition["flows"]}
    arrived_entries = {entry_of.get(trip.get("id", "").rsplit(".", 1)[0]) for trip in arrived}
    entries_without_arrival = sorted(set(entry_of.values()) - arrived_entries)
    lanes = {lane.get("id"): lane for lane in ET.parse(folder / "lanes.xml").getroot().iter("lane")}
    collisions = len(ET.parse(folder / "collisions.xml").getroot().findall("collision"))
    return {
        "scenario": scenario, "seed": seed,
        "ok": collisions == 0 and teleport_count == 0 and not entries_without_arrival,
        "collisions": collisions, "teleports": teleport_count,
        "emergency_braking": _number(statistics.find("safety"), "emergencyBraking"),
        "mean_running": round(sum(_number(row, "running") for row in window) / len(window), 3),
        "max_trip_waiting_s": max((float(trip.get("waitingTime", 0)) for trip in trips), default=0.0),
        "arrived": len(arrived), "entries_without_arrival": entries_without_arrival,
        "lanes": {lane_id: ({key: float(lanes[lane_id].get(key)) for key in ("speed", "occupancy", "waitingTime")
                             if lanes[lane_id].get(key) is not None} if lane_id in lanes else None)
                  for lane_id in SCENARIO_LANES},
    }


def run_ego(folder: Path, scenario: str, seed: int, route: str) -> dict:
    """경로 추종 규칙으로 ego 한 에피소드를 달려 투입·종료·정지·보상을 잰다."""
    import traci
    from env.mdp_config import OBSERVATION, ACTION, REWARD
    from env.sumo_env import SumoHighwayEnv

    configure_map(scenario, seed, route)
    cfg = road_builder.build(ROAD, EGO, TRAFFIC, str(folder))
    step = float(SIMULATION["step_length"])
    decision = float(SIMULATION["decision_interval"])
    row = {"route": route, "scenario": scenario, "seed": seed}
    env = SumoHighwayEnv(cfg_path=cfg, road=ROAD, ego=EGO, mdp_sim=SIMULATION,
                         mdp_obs=OBSERVATION, action=ACTION, reward=REWARD, gui=False)
    try:
        try:
            observation, info = env.reset(seed=seed)
        except RuntimeError as error:
            row.update(ok=False, termination_reason="insertion_failed", error=str(error)[:200])
            return row
        row["insertion_wait_s"] = round(info["ego_insert_wait_seconds"] - step, 1)
        total, streak, longest = 0.0, 0, 0
        for index in range(env.max_steps):
            observation, reward, terminated, truncated, info = env.step(route_following_action(observation))
            total += float(reward)
            if env.ego_id in traci.vehicle.getIDList() and traci.vehicle.getSpeed(env.ego_id) < 0.1:
                streak += 1
                longest = max(longest, streak)
            else:
                streak = 0
            if terminated or truncated:
                break
        reason = info["termination_reason"]
        expected = ("arrived",) if route == ROAD["network"]["ego_route"] else SCENARIO_OUTCOMES
        row.update(ok=not info["collided"] and reason in expected,
                   termination_reason=reason, episode_s=round((index + 1) * decision, 1),
                   longest_stop_s=round(longest * decision, 1), total_reward=round(total, 3))
    finally:
        env.close()
    return row


def run_job(kind: str, folder: str, scenario: str, seed: int, seconds: float, route: str | None) -> dict:
    """프로세스 풀의 한 작업. 예외도 행으로 돌려 보내 나머지 결과가 사라지지 않게 한다."""
    try:
        if kind == "background":
            return run_background(Path(folder), scenario, seed, seconds)
        return run_ego(Path(folder), scenario, seed, route)
    except Exception as error:  # noqa: BLE001
        return {"route": route, "scenario": scenario, "seed": seed, "ok": False,
                "error": f"{type(error).__name__}: {str(error)[-500:]}"}


def print_tables(report: dict) -> None:
    print("배경  scenario  seed  충돌  순간이동  급제동  평균주행  최대대기(s)  결과")
    for row in report["background"]:
        print(f"      {row['scenario']:9s} {row['seed']:<5d} {row.get('collisions', '-'):<5} {row.get('teleports', '-'):<9} "
              f"{row.get('emergency_braking', '-'):<7} {row.get('mean_running', '-'):<9} {row.get('max_trip_waiting_s', '-'):<12} "
              f"{'통과' if row['ok'] else '실패 ' + row.get('error', '')}")
    print("ego   route                   scenario  seed  투입대기(s)  종료사유          길이(s)  최장정지(s)  보상합    결과")
    for row in report["ego"]:
        print(f"      {row['route']:23s} {row['scenario']:9s} {row['seed']:<5d} {row.get('insertion_wait_s', '-'):<12} "
              f"{row.get('termination_reason', '-'):17s} {row.get('episode_s', '-'):<8} {row.get('longest_stop_s', '-'):<12} "
              f"{row.get('total_reward', '-'):<9} {'통과' if row['ok'] else '실패 ' + row.get('error', '')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 7, 42])
    parser.add_argument("--scenarios", nargs="+", choices=("quiet", "baseline", "busy"), default=["quiet", "baseline", "busy"])
    parser.add_argument("--seconds", type=float, default=1800)
    parser.add_argument("--out", type=Path, required=True, help="요약 JSON을 쓸 경로")
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    if args.seconds < 600:
        parser.error("차량 대기와 순간이동을 관찰할 수 있도록 --seconds는 600 이상으로 지정하세요.")
    configure_map("baseline", 0)
    definition = ROAD["network"]
    structure = validate_structure(definition, Path(definition["net_path"]))
    ego_routes = [definition["ego_route"]] + sorted(route for route in definition["routes"] if route.startswith("s_"))
    report = {
        "command": shlex.join(["python", *sys.argv]),
        "map_version": definition["version"], "network_sha256": definition["network_source"]["sha256"],
        "sumo_version": subprocess.run([checkBinary("sumo"), "--version"], capture_output=True, text=True,
                                       check=True).stdout.splitlines()[0],
        "step_length": SIMULATION["step_length"], "seconds": args.seconds,
        "warmup_seconds": SIMULATION["warmup_steps"] * SIMULATION["step_length"],
        "max_episode_seconds": SIMULATION["max_steps"] * SIMULATION["decision_interval"],
        "lane_data_from_seconds": LANE_DATA_BEGIN_S, "scenario_lanes": SCENARIO_LANES,
        "pass_condition": PASS_CONDITION, "structure": structure,
        "ok": False, "background": [], "ego": [],
    }
    with tempfile.TemporaryDirectory(prefix="paldalmun_validation_") as temporary, \
            ProcessPoolExecutor(max_workers=args.workers) as pool:
        jobs = []
        for scenario in args.scenarios:
            for seed in args.seeds:
                jobs.append(pool.submit(run_job, "background", f"{temporary}/background/{scenario}/{seed}",
                                        scenario, seed, args.seconds, None))
                for route in ego_routes:
                    jobs.append(pool.submit(run_job, "ego", f"{temporary}/ego/{route}/{scenario}/{seed}",
                                            scenario, seed, args.seconds, route))
        for done, future in enumerate(as_completed(jobs), start=1):
            row = future.result()
            report["ego" if "route" in row and row["route"] else "background"].append(row)
            print(f"[{done}/{len(jobs)}] {row.get('route') or '배경'} {row['scenario']} seed {row['seed']}: "
                  f"{'통과' if row['ok'] else '실패'}", flush=True)
    order = {scenario: index for index, scenario in enumerate(args.scenarios)}
    report["background"].sort(key=lambda row: (order[row["scenario"]], row["seed"]))
    report["ego"].sort(key=lambda row: (ego_routes.index(row["route"]), order[row["scenario"]], row["seed"]))
    report["ok"] = all(row["ok"] for row in report["background"] + report["ego"])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print_tables(report)
    print(f"{'통과' if report['ok'] else '실패'}: {args.out}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
