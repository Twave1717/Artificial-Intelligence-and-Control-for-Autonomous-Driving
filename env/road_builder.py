"""팔달문 도로를 SUMO가 읽을 실행 파일로 만든다.

지도 원본(maps/paldalmun/)은 수정하지 않고 out_dir 안에만 파일을 만든다.
학습·평가·수집·검증이 같은 빌더를 쓴다.

    network.net.xml   검수한 native 도로망의 바이트 단위 사본
    routes.rou.xml    차량 타입, 경로, 유입 수요
    map.add.xml       배경 폴리곤과 정류장 시설(busStop). 버스·승객은 없다
    view.gui.xml      sumo-gui 초기 시점
    scenario.sumocfg  위 파일을 묶는 실행 진입점
    manifest.json     지도 버전·해시와 이번 실행의 설정
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from sumolib import checkBinary
from sumolib.net import readNet

from env.mdp_config import SIMULATION
from maps import MAPS_DIR


def _write_xml(path: Path, root: ET.Element) -> None:
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _attrs(values: dict) -> dict:
    return {str(key): str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in values.items() if value is not None}


def _write_vehicle_types(routes: ET.Element, ego: dict, traffic: dict) -> None:
    """ego는 승용차 한 종류, 배경차는 추종모델 네 종류의 확률 분포다."""
    lateral = {attribute: traffic["lateral_model"][key] for key, attribute in (
        ("accel", "lcAccelLat"), ("max_speed", "maxSpeedLat"),
        ("standing_speed", "lcMaxSpeedLatStanding"), ("speed_factor", "lcMaxSpeedLatFactor"),
        ("roundabout_cooperation", "lcCooperativeRoundabout"))}
    if float(traffic["lateral_resolution"]) > 0:
        lateral["laneChangeModel"] = "SL2015"
    ET.SubElement(routes, "vType", _attrs(dict(
        id="ego_type", accel=ego["accel"], decel=ego["decel"], maxSpeed=ego["max_speed"],
        speedFactor=ego["speed_factor"], speedDev=ego["speed_dev"], color="1,0,0", vClass="passenger",
        guiShape="passenger/sedan", **lateral)))
    controllers = traffic["controllers"]
    total = sum(float(c["probability"]) for c in controllers.values())
    distribution = ET.SubElement(routes, "vTypeDistribution", id="carDist")
    for name, c in controllers.items():
        # 도시에서는 회전 경로를 따라갈 차선변경이 필요하다. SUMO의 기본
        # 전략/협조 차선변경을 유지한다.
        ET.SubElement(distribution, "vType", _attrs(dict(
            id=f"car_{name}", probability=float(c["probability"]) / total,
            carFollowModel=c["carFollowModel"], accel=c["accel"], decel=c["decel"],
            tau=c["tau"], minGap=c["min_gap"], maxSpeed=traffic["max_speed"],
            speedDev=c["speed_dev"], color=c["color"], sigma=c.get("sigma"),
            collisionAvoidanceOverride=c.get("collision_avoidance_override"),
            vClass="passenger", guiShape="passenger/sedan", **lateral)))


def build(road: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """실행 파일을 out_dir에 만들고 scenario.sumocfg 경로를 반환한다."""
    network = road["network"]
    output = Path(out_dir).expanduser().resolve()
    if output == MAPS_DIR or MAPS_DIR in output.parents:
        raise ValueError("out_dir는 maps/ 밖의 실행 폴더로 지정하세요. 지도 원본은 덮어쓰지 않습니다.")
    output.mkdir(parents=True, exist_ok=True)

    # 1) 검수한 native 도로망을 바이트 그대로 복사한다. 실행 때 netconvert를
    #    다시 돌리면 교차로 곡선이 SUMO 버전에 따라 달라질 수 있다. 경로와 정류장이
    #    이 도로망에 맞는지는 scripts/build_road.py가 게시 전에 검사했다.
    content = Path(network["net_path"]).read_bytes()
    network_sha256 = hashlib.sha256(content).hexdigest()
    if network_sha256 != network["network_source"]["sha256"]:
        raise ValueError("native 도로망의 해시가 map.json과 다릅니다.")
    (output / "network.net.xml").write_bytes(content)
    compiled = readNet(str(output / "network.net.xml"))

    # 2) 차량 타입, 경로, 유입 수요
    routes = ET.Element("routes")
    _write_vehicle_types(routes, ego, traffic)
    for route_id, edge_ids in network["routes"].items():
        ET.SubElement(routes, "route", id=route_id, edges=" ".join(edge_ids))
    scale = float(traffic["demand_scale"])
    # 같은 진입로의 흐름들이 같은 시각에 차를 넣으면 겹쳐 투입되어 충돌한다. 진입로 전체의 투입 간격으로
    # 시작 시각을 엇갈려, 진입로마다 차가 고른 간격으로 들어오게 한다.
    entry_total, entry_seen = {}, {}
    for flow in network["flows"]:
        entry_total[flow["from"]] = entry_total.get(flow["from"], 0.0) + float(flow["vehs_per_hour"]) * scale
    staggered = []
    for flow in network["flows"]:
        volume = float(flow["vehs_per_hour"]) * scale
        if volume > 0:
            staggered.append((entry_seen.get(flow["from"], 0) * 3600.0 / entry_total[flow["from"]], volume, flow))
            entry_seen[flow["from"]] = entry_seen.get(flow["from"], 0) + 1
    # SUMO는 시작 시각 순서가 아닌 흐름을 무시한다.
    for begin, volume, flow in sorted(staggered, key=lambda item: item[0]):
        # 배경 차량은 출발·도착 도로만 정하고 경로는 SUMO가 고른다.
        ET.SubElement(routes, "flow", _attrs({
            "id": flow["id"], "type": "carDist", "from": flow["from"], "to": flow["to"], "begin": f"{begin:.3f}",
            "end": 100000, "period": f"{3600.0 / volume:.6f}", "departLane": "best",
            "departSpeed": traffic["depart_speed"]}))
    _write_xml(output / "routes.rou.xml", routes)

    # 3) 정류장은 도로 시설로만 표시한다. 버스와 승객은 넣지 않는다.
    additional = ET.parse(network["context_path"]).getroot()
    for stop in network.get("bus_stops", []):
        ET.SubElement(additional, "busStop", _attrs(dict(
            id=stop["id"], name=stop["name"], lane=stop["lane"],
            startPos=stop["startPos"], endPos=stop["endPos"])))
    _write_xml(output / "map.add.xml", additional)

    # 4) GUI 초기 시점: 관심 영역(focus_bbox)이 화면에 맞도록 확대한다.
    left, bottom, right, top = compiled.getBoundary()
    west, south, east, north = network["focus_bbox"]
    x0, y0 = compiled.convertLonLat2XY(west, south)
    x1, y1 = compiled.convertLonLat2XY(east, north)
    zoom = 100.0 * max(right - left, top - bottom) / max(abs(x1 - x0), abs(y1 - y0))
    view = ET.Element("viewsettings")
    ET.SubElement(view, "scheme", name="real world")
    ET.SubElement(view, "viewport", _attrs(dict(
        zoom=zoom, x=network["gui_center"]["x"], y=network["gui_center"]["y"], angle=0)))
    ET.SubElement(view, "delay", value=str(SIMULATION["gui_delay"]))
    _write_xml(output / "view.gui.xml", view)

    # 5) 교차로 내부의 충돌도 집계한다. 평가 때도 같은 판정 기준을 쓴다.
    configuration = ET.Element("configuration")
    inputs = ET.SubElement(configuration, "input")
    for name, value in (("net-file", "network.net.xml"), ("route-files", "routes.rou.xml"),
                        ("additional-files", "map.add.xml")):
        ET.SubElement(inputs, name, value=value)
    processing = ET.SubElement(configuration, "processing")
    for name, value in (("lateral-resolution", traffic["lateral_resolution"]),
                        ("collision.check-junctions", True), ("collision.action", "remove"),
                        ("collision.mingap-factor", traffic["collision_min_gap_factor"])):
        ET.SubElement(processing, name, _attrs(dict(value=value)))
    ET.SubElement(ET.SubElement(configuration, "time"), "step-length",
                  value=str(SIMULATION["step_length"]))
    ET.SubElement(ET.SubElement(configuration, "random_number"), "seed", value=str(road["seed"]))
    ET.SubElement(ET.SubElement(configuration, "gui_only"), "gui-settings-file", value="view.gui.xml")
    cfg_path = output / "scenario.sumocfg"
    _write_xml(cfg_path, configuration)

    # 6) 지도 버전만으로는 내용이 같은지 알 수 없다. 원본과 생성 파일의 해시를 함께 남긴다.
    generated = ["network.net.xml", "routes.rou.xml", "map.add.xml", "view.gui.xml", "scenario.sumocfg"]
    sumo_version = subprocess.run([checkBinary("sumo"), "--version"], capture_output=True,
                                  text=True, check=True).stdout.splitlines()[0]
    manifest = dict(
        map_id=network["id"], map_version=network["version"], network_sha256=network_sha256,
        scenario=road["scenario"], seed=road["seed"], demand_scale=scale, ego=ego, traffic=traffic,
        generated_sha256={name: hashlib.sha256((output / name).read_bytes()).hexdigest()
                          for name in generated},
        sumo_version=sumo_version, step_length=SIMULATION["step_length"])
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    return str(cfg_path)
