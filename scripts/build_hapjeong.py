"""합정역 실행 도로망과 부속 파일, map.json을 만든다.

    python scripts/build_hapjeong.py

입력은 서예인이 maps/hapjeong/generator/의 생성기로 만든 원본(source/network.base.net.xml,
source/signals.base.add.xml)이다. 임시 폴더에서 다음을 모두 마친 뒤에만 결과를 바꿔 넣는다.
1. 제한속도를 실제 기준으로 낮춘다(간선 50km/h, 연결로·"~길" 이면도로 30km/h, 더 낮은 값은 그대로).
2. 11m 간격으로 붙어 차가 교차로 안에 갇히던 J52·J56·J58을 한 교차로로 합치고 횡단보도를 다시 놓는다.
3. 신호 프로그램에서 합친 교차로의 J56 프로그램을 빼고(기본 신호 사용), J1에서 중앙버스차로가
   일반 차로로 합류하는 연결을 양보 녹색(g)으로 바꾼다. 같은 차로에 우선 녹색 두 개가 들어가던 곳이다.
4. 바뀐 도로망에서 보행자 경로를 원본과 같은 설정(시간당 600명, 최소 150m)과 고정 seed로 다시 만든다.
5. 독막로 북쪽의 이어진 골목 세 개(양화로6길 서쪽·동쪽, 독막로3길)를 OSM 좌표로 붙인다.
   양화로6길 양 끝은 도로망 경계이고, 독막로3길이 독막로에 신호 없이 합류한다(S5).
6. 연구 시나리오(S1~S3, S6)용으로 OSM 위치의 가로변 정류장 6곳과 와우산로의 신호 없는 횡단보도 1곳을 둔다.
7. 배경 흐름(팔달문과 같은 규칙을 합계 900대/시간으로 줄임)과 ego 경로를 계산하고 validate_structure로 확인한다.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_road import boundary_flows, publish, shortest_route
from scripts.patch_network import digest
from scripts.prune_network import OPTIONS
from scripts.validate_road import validate_structure

FOLDER = BASE / "maps" / "hapjeong"
SOURCE = FOLDER / "source"
VERSION = "2.0.0"
ARTERIAL_KMH, LINK_KMH, ALLEY_KMH = 50, 30, 30
LINK_EDGES = {"R65", "R65.c", "R66", "R66.c", "R67", "R67.c"}   # 생성기 설정에서 primary_link인 연결로
JOIN = [("J52", "J56", "J58")]
# 연결로가 양화로에 지퍼로 합류하던 J3은 busy에서 정체가 풀리지 않아, 연결로가 양보하는 합류로 바꾼다.
NODE_TYPES = {"J3": "priority"}
JOINED = "cluster_J52_J56_J58"
JOINED_CROSSINGS = [("R73", "-R73"), ("R77", "-R77"), ("R120",), ("R122",), ("R121",), ("R123", "-R123")]
YIELD_LINKS = {"J1": [0, 11]}          # B2_0 → R40_3, B1_0 → R41_2
DROP_PROGRAMS = {"J56"}
# 배경 차량 baseline 합계. 생성기 설정(generator/road_config.py OSM_ROAD traffic)의 값이다.
BASELINE_VEHS_PER_HOUR = 900
PEDESTRIANS = {"persons_per_hour": 600, "min_distance": 150, "end": 3600, "seed": 42}
# 신호 없는 횡단보도(S6). OSM 13236303911(crossing=uncontrolled)은 와우산로 차도 위의 점이다.
MIDBLOCK_CROSSINGS = {"S6_wausanro": ("R112", "-R112", 37.5498511, 126.9231277)}
# 연구 경로의 가로변 정류장(OSM 노드, 이름, ARS 번호, 위도, 경도, 놓을 도로). 위치는 Overpass에서 2026-11-03에 받았다.
CURB_STOPS = [
    ("357920065", "합정역7번출구", "14-119", 37.5486897, 126.9142951, "R69"),
    ("357920024", "성산중학교", "", 37.5480489, 126.9174405, "R71"),
    ("357916587", "상수역", "", 37.5476100, 126.9212700, "R111"),
    ("5930628870", "극동방송", "", 37.5495471, 126.9231077, "R112"),
    ("3446267493", "서교동교회", "14-984", 37.5517944, 126.9193584, "-R96"),
    ("3424857165", "합정역", "14-219", 37.5510384, 126.9157425, "R54a"),
]
# 독막로 북쪽의 이어진 골목(OSM 172603991 양화로6길, 172603989 독막로3길). 양화로6길 양 끝은 도로망 경계라
# 골목 차가 드나들고, 독막로3길이 두 골목을 독막로에 신호 없이 잇는다(S5). J8 신호 교차로에는 잇지 않는다.
# 서비스도로로 J8에 이으면 J8에서 골목으로 꺾는 차가 늘어 충돌과 순간이동이 났다.
ALLEY_SPLITS = {"X_dongmak3": ("R71", "-R71", 37.5483458, 126.9163868)}   # OSM 1834932797, 독막로3길이 독막로에 닿는 점
ALLEY_NODES = {
    "Y2": (37.5488709, 126.9165847, "priority"),          # OSM 3066579587, 양화로6길과 독막로3길
    "D_yanghwa6": (37.5493883, 126.9148575, "dead_end"),  # OSM 2374776990, 양화로6길 서쪽 끝(도로망 경계)
    "E_yanghwa6": (37.5486818, 126.9174365, "dead_end"),  # OSM 1834932800, 양화로6길 동쪽 끝(도로망 경계)
}
# (도로 id, 이름, 시작 노드, 끝 노드, 중간점 (위도, 경도), km/h). 반대 방향은 "-" + id로 만든다.
ALLEYS = [
    ("G2", "양화로6길", "D_yanghwa6", "Y2", [(37.5493377, 126.9149078), (37.5492114, 126.9150487), (37.5491389, 126.9153241),
                                           (37.5491115, 126.9154291), (37.5490346, 126.9157431), (37.5489996, 126.9159087),
                                           (37.5489393, 126.9161962)], ALLEY_KMH),
    ("G2e", "양화로6길", "Y2", "E_yanghwa6", [(37.5487897, 126.9169195), (37.5487566, 126.9170802)], ALLEY_KMH),
    ("G3", "독막로3길", "Y2", "X_dongmak3", [(37.5484422, 126.9164129)], ALLEY_KMH),
]
# 새 골목의 양 끝은 배경 차량의 출발지로만 쓴다. 도착지로도 쓰면 모든 진입로가 차를 똑같이 나눠 보내
# 골목으로 시간당 약 160대가 몰렸고, 합정역 쪽 독막로가 막혀 J8 횡단보도 위에 차가 섰다.
ORIGIN_ONLY_EXITS = {"-G2", "G2e"}
EGO_ROUTE = "bus_loop"
# (설명, 경유 도로). 서예인이 제안한 순환 경로다. 상수역 정차 뒤 약 140m 안에 와우산로로 좌회전한다(S2).
ROUTES = {EGO_ROUTE: ("[연구] 합정역 → 성산중학교 → 상수역 → 와우산로 좌회전 → 극동방송 → 잔다리로 → 서교동교회 → "
                      "홍대입구역 교차로 → 양화로 → 합정역",
                      ["R69", "R71", "R111", "R112", "R113", "-R96", "R94", "R54a", "R54", "R42"])}
EXTRA_FILES = {
    "additional": ["source/signals.add.xml", "source/islands.add.xml", "source/seams.add.xml", "source/stops.add.xml"],
    "routes": ["source/pedestrians.rou.xml"],
}


def netconvert(args: list[str], output: Path) -> None:
    completed = subprocess.run([sumolib.checkBinary("netconvert"), *args, *OPTIONS, "--output-file", str(output)],
                               capture_output=True, text=True, timeout=300)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    # 머리 주석의 생성 시각과 임시 경로를 지워 빌드마다 같은 바이트가 되게 한다.
    ET.ElementTree(ET.parse(output).getroot()).write(output, encoding="utf-8", xml_declaration=True)


def build_network(temporary: Path) -> Path:
    base = sumolib.net.readNet(str(SOURCE / "network.base.net.xml"))
    edges = ET.Element("edges")
    patches = {}
    for edge in base.getEdges():
        # 이름이 "~길"인 이면도로는 30km/h다. 50km/h 골목 토막 끝에서 출발한 차가 바로 앞 횡단보도에서 멈추지 못했다.
        kmh = LINK_KMH if edge.getID() in LINK_EDGES else ALLEY_KMH if edge.getName().endswith("길") else ARTERIAL_KMH
        target = kmh / 3.6
        if max(lane.getSpeed() for lane in edge.getLanes()) > target + 0.01:
            patches[edge.getID()] = ET.SubElement(edges, "edge", id=edge.getID(), speed=f"{target:.2f}")
    for node_id, (forward, backward, lat, lon) in {**MIDBLOCK_CROSSINGS, **ALLEY_SPLITS}.items():
        point = base.convertLonLat2XY(lon, lat)
        for edge_id in (forward, backward):
            position, _ = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(point, base.getEdge(edge_id).getShape())
            element = patches.get(edge_id) or ET.SubElement(edges, "edge", id=edge_id)
            ET.SubElement(element, "split", pos=f"{position:.2f}", id=node_id)
    nodes = ET.Element("nodes")
    for group in JOIN:
        ET.SubElement(nodes, "join", nodes=" ".join(group))
    for node_id, node_type in NODE_TYPES.items():
        ET.SubElement(nodes, "node", id=node_id, type=node_type)
    ET.ElementTree(edges).write(temporary / "speed.edg.xml", encoding="utf-8")
    ET.ElementTree(nodes).write(temporary / "join.nod.xml", encoding="utf-8")
    joined = temporary / "joined.net.xml"
    netconvert(["--sumo-net-file", str(SOURCE / "network.base.net.xml"), "--edge-files", str(temporary / "speed.edg.xml"),
                "--node-files", str(temporary / "join.nod.xml")], joined)
    crossings = ET.Element("connections")
    for group in JOINED_CROSSINGS:
        ET.SubElement(crossings, "crossing", node=JOINED, edges=" ".join(group))
    split_net = sumolib.net.readNet(str(joined))
    for node_id, (forward, _, _, _) in MIDBLOCK_CROSSINGS.items():
        node = split_net.getNode(node_id)
        incoming = next(edge for edge in node.getIncoming() if edge.getID() == forward)
        outgoing = next(edge for edge in node.getOutgoing() if edge.getToNode() == incoming.getFromNode())
        # 신호 없는 횡단보도에서는 차가 보행자에게 양보한다.
        ET.SubElement(crossings, "crossing", node=node_id, edges=f"{incoming.getID()} {outgoing.getID()}", priority="true")
    alley_nodes, alley_edges = add_alleys(split_net, crossings)
    ET.ElementTree(crossings).write(temporary / "crossings.con.xml", encoding="utf-8")
    ET.ElementTree(alley_nodes).write(temporary / "alleys.nod.xml", encoding="utf-8")
    ET.ElementTree(alley_edges).write(temporary / "alleys.edg.xml", encoding="utf-8")
    output = temporary / "network.net.xml"
    netconvert(["--sumo-net-file", str(joined), "--node-files", str(temporary / "alleys.nod.xml"),
                "--edge-files", str(temporary / "alleys.edg.xml"), "--connection-files", str(temporary / "crossings.con.xml")],
               output)
    return output


def add_alleys(net, connections: ET.Element) -> tuple[ET.Element, ET.Element]:
    """골목 노드·도로를 만들고, 독막로를 나눠 만든 노드(X_dongmak3)에서 새 골목과 이어지는 연결을 차로까지 적는다.

    도로망 파일에서 읽은 노드의 연결은 고정값이라, 적지 않으면 새 골목과 이어지지 않는다.
    """
    nodes, edges = ET.Element("nodes"), ET.Element("edges")
    coords = {node_id: net.convertLonLat2XY(lon, lat) for node_id, (lat, lon, _) in ALLEY_NODES.items()}
    for node_id, (_, _, node_type) in ALLEY_NODES.items():
        ET.SubElement(nodes, "node", id=node_id, x=f"{coords[node_id][0]:.3f}", y=f"{coords[node_id][1]:.3f}", type=node_type)
    for node_id in ALLEY_SPLITS:
        coords[node_id] = net.getNode(node_id).getCoord()
    new = {}
    for edge_id, name, start, end, via, kmh in ALLEYS:
        shape = [coords[start], *(net.convertLonLat2XY(lon, lat) for lat, lon in via), coords[end]]
        for direction, (a, b, points) in {edge_id: (start, end, shape), "-" + edge_id: (end, start, shape[::-1])}.items():
            new[direction] = (a, b)
            ET.SubElement(edges, "edge", {"id": direction, "from": a, "to": b, "name": name, "numLanes": "1",
                                          "speed": f"{kmh / 3.6:.2f}", "priority": "1", "disallow": "pedestrian",
                                          "shape": " ".join(f"{x:.3f},{y:.3f}" for x, y in points)})

    def drivable(edge, outermost=True):
        lanes = [lane.getIndex() for lane in edge.getLanes() if lane.allows("passenger")]
        return str(lanes[0] if outermost else lanes[-1])
    for node_id in ALLEY_SPLITS:
        node = net.getNode(node_id)
        for target, (a, b) in new.items():
            if a == node_id:
                for source in node.getIncoming():
                    if source.getFromNode().getID() != b:
                        # 우회전은 바깥 차로, 좌회전·직진은 안쪽 차로에서 골목으로 들어간다.
                        (x0, y0), (x1, y1) = source.getShape()[-2:]
                        x2, y2 = coords[b]
                        right = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) < 0
                        ET.SubElement(connections, "connection", {"from": source.getID(), "to": target,
                                                                  "fromLane": drivable(source, right), "toLane": "0"})
            if b == node_id:
                for sink in node.getOutgoing():
                    if sink.getToNode().getID() != a:
                        ET.SubElement(connections, "connection", {"from": target, "to": sink.getID(),
                                                                  "fromLane": "0", "toLane": drivable(sink)})
    return nodes, edges


def build_signals(net_path: Path, output: Path) -> None:
    root = ET.parse(SOURCE / "signals.base.add.xml").getroot()
    tls_ids = {tl.getID() for tl in sumolib.net.readNet(str(net_path)).getTrafficLights()}
    for logic in list(root.findall("tlLogic")):
        if logic.get("id") in DROP_PROGRAMS:
            root.remove(logic)
            continue
        if logic.get("id") not in tls_ids:
            raise ValueError(f"신호 프로그램 {logic.get('id')}의 교차로가 도로망에 없습니다.")
        for phase in logic.findall("phase"):
            state = list(phase.get("state"))
            for index in YIELD_LINKS.get(logic.get("id"), []):
                state[index] = "g" if state[index] == "G" else state[index]
            phase.set("state", "".join(state))
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def build_pedestrians(net_path: Path, output: Path, temporary: Path) -> None:
    tool = Path(os.environ["SUMO_HOME"]) / "tools" / "randomTrips.py"
    period = 3600 / PEDESTRIANS["persons_per_hour"]
    completed = subprocess.run(
        [sys.executable, str(tool), "-n", str(net_path), "--pedestrians", "-b", "0", "-e", str(PEDESTRIANS["end"]),
         "-p", f"{period:.4f}", "--min-distance", str(PEDESTRIANS["min_distance"]), "--seed", str(PEDESTRIANS["seed"]),
         "--prefix", "ped", "-o", str(temporary / "pedestrians.trips.xml"), "-r", str(output)],
        capture_output=True, text=True, timeout=600, cwd=temporary)
    if completed.returncode:
        raise RuntimeError("randomTrips 실패: " + completed.stderr[-3000:])
    ET.ElementTree(ET.parse(output).getroot()).write(output, encoding="utf-8", xml_declaration=True)


def curb_stops(net) -> list[dict]:
    """OSM 정류장을 지정한 도로(나뉜 경우 그 조각 포함)의 가장 바깥 차도 차로에 25m 길이로 놓는다."""
    stops = []
    for osm_id, name, ref, lat, lon, edge_id in CURB_STOPS:
        point = net.convertLonLat2XY(lon, lat)
        candidates = []
        for edge in net.getEdges():
            if edge.getID() != edge_id and not edge.getID().startswith(edge_id + "."):
                continue
            lane = next(lane for lane in edge.getLanes() if lane.allows("bus"))
            offset, distance = sumolib.geomhelper.polygonOffsetAndDistanceToPoint(point, lane.getShape())
            candidates.append((distance, offset, lane))
        distance, offset, lane = min(candidates, key=lambda item: item[0])
        if distance > 15:  # OSM 정류장 점은 인도 위 승강장에 찍혀 차로에서 10m 넘게 떨어지기도 한다
            raise ValueError(f"정류장 {name}({osm_id})이 {edge_id}에서 {distance:.1f}m 떨어져 있습니다.")
        end = min(max(offset + 12.5, 25.5), lane.getLength() - 0.5)
        stops.append({"id": f"osm_stop_{osm_id}", "name": name, "ref": ref or None, "lane": lane.getID(),
                      "startPos": round(end - 25, 2), "endPos": round(end, 2),
                      "lanes_on_edge": sum(item.allows("bus") for item in lane.getEdge().getLanes()),
                      "source_osm_node": osm_id, "latitude": lat, "longitude": lon,
                      "snap_distance_m": round(distance, 3)})
    return stops


def scaled_flows(net) -> list[dict]:
    """팔달문과 같은 경계 흐름 규칙으로 나눈 뒤, 합계가 BASELINE_VEHS_PER_HOUR가 되게 줄인다."""
    flows = [flow for flow in boundary_flows(net) if flow["to"] not in ORIGIN_ONLY_EXITS]
    scale = BASELINE_VEHS_PER_HOUR / sum(flow["vehs_per_hour"] for flow in flows)
    return [{**flow, "vehs_per_hour": round(flow["vehs_per_hour"] * scale, 3)} for flow in flows]


def build() -> dict:
    with tempfile.TemporaryDirectory(prefix="hapjeong_build_") as name:
        temporary = Path(name)
        net_path = build_network(temporary)
        build_signals(net_path, temporary / "signals.add.xml")
        build_pedestrians(net_path, temporary / "pedestrians.rou.xml", temporary)
        net = sumolib.net.readNet(str(net_path))
        routes, route_design = {}, {}
        for route_id, (description, anchors) in ROUTES.items():
            edges = routes[route_id] = shortest_route(net, anchors, route_id)
            lanes = [lane for edge in edges for lane in net.getEdge(edge).getLanes() if lane.allows("passenger")]
            route_design[route_id] = {
                "description": description,
                "length_m": round(sum(net.getEdge(edge).getLength() for edge in edges), 2),
                "max_speed_mps": max(lane.getSpeed() for lane in lanes),
                "first_edge_lanes": sum(lane.allows("passenger") for lane in net.getEdge(edges[0]).getLanes()),
            }
        (west, south), (east, north) = (net.convertXY2LonLat(*corner) for corner in
                                        (net.getBoundary()[:2], net.getBoundary()[2:]))
        center = net.convertLonLat2XY((west + east) / 2, (south + north) / 2)
        definition = {
            "id": "hapjeong", "name": "합정역 · 양화로와 독막로", "version": VERSION,
            "description": "합정역 일대 실제 도로. 신호 교차로, 인도·횡단보도·보행자, 중앙버스차로, 연구 경로의 가로변 정류장 6곳과 신호 없는 횡단보도가 있다.",
            "source_status": "osm_generated",
            "network_source": {"path": "source/network.net.xml", "sha256": digest(net_path),
                               "base_path": "source/network.base.net.xml", "base_sha256": digest(SOURCE / "network.base.net.xml"),
                               "generator": "generator/ (서예인, 합정역 브랜치 ec6fbf6에서 옮김)",
                               "patches": {"speed_kmh": {"arterial": ARTERIAL_KMH, "link": LINK_KMH, "alley": ALLEY_KMH},
                                           "joined_junctions": [list(group) for group in JOIN],
                                           "signal_yield_links": YIELD_LINKS, "dropped_programs": sorted(DROP_PROGRAMS)},
                               "pedestrians": PEDESTRIANS},
            "extra_files": EXTRA_FILES,
            "trainable": False,
            "trainable_reason": "일반 도로 대부분의 0번 차로가 인도라서, 차로 번호를 관측으로 쓰는 학습 환경이 틀린 값을 읽는다.",
            "focus_bbox": [round(west, 7), round(south, 7), round(east, 7), round(north, 7)],
            "gui_center": {"x": round(center[0], 3), "y": round(center[1], 3)},
            "sources_license": {"road_data": "ODbL 1.0", "attribution": "© OpenStreetMap contributors",
                                "url": "https://www.openstreetmap.org/copyright"},
            "assumptions": ["제한속도는 안전속도 5030 기준이다. 간선 50km/h, 연결로와 이름이 ~길인 이면도로 30km/h이고 OSM의 독막로 50km/h와 같다.",
                            "배경 차량 유입량은 팔달문과 같은 규칙으로 나눈 합성 수요다. quiet·baseline·busy는 0.5·1·1.25배다.",
                            "배경 차량 baseline 합계는 생성기 설정의 시간당 900대다."],
            "routes": routes, "route_design": route_design, "flows": scaled_flows(net),
            "max_lane_speed_mps": max(lane.getSpeed() for edge in net.getEdges() for lane in edge.getLanes()
                                      if lane.allows("passenger")),
            "ego_route": EGO_ROUTE, "bus_stops": curb_stops(net),
            "design": {"measure_lanes": [], "zones": {}},
        }
        validate_structure(definition, net_path)
        publish({SOURCE / "network.net.xml": net_path.read_bytes(),
                 SOURCE / "signals.add.xml": (temporary / "signals.add.xml").read_bytes(),
                 SOURCE / "pedestrians.rou.xml": (temporary / "pedestrians.rou.xml").read_bytes(),
                 FOLDER / "map.json": (json.dumps(definition, ensure_ascii=False, indent=2) + "\n").encode("utf-8")})
    return definition


if __name__ == "__main__":
    result = build()
    print(f"hapjeong {result['version']}: 배경 흐름 {len(result['flows'])}개, "
          f"baseline {round(sum(flow['vehs_per_hour'] for flow in result['flows']))}대/시간, "
          f"실행망 sha256 {result['network_source']['sha256'][:12]}")
