"""팔달문 실행 도로망과 map.json을 만든다.

    python scripts/build_road.py

임시 폴더 안에서 다음을 모두 마친 뒤에만 source/network.net.xml과 map.json 두 파일을 바꿔 넣는다.
1. scripts/patch_network.py로 source/network.full.net.xml에서 신호를 걷어내고
   겹친 분할점을 합친 설계망을 만들고 원본과 대조한다.
   설계망에 없는 ADDED_OSM_WAYS(OSM service 길)를 OSM 좌표 그대로 붙인다.
2. source/drawn_edges.json의 도로(팀이 지도에 표시한 범위)만 남긴 실행망을 scripts/prune_network.py로
   만든다. 도로 형상과 교차로 좌표가 설계망과 다르면 거절한다.
3. 아래 ROUTES(ego 경로)의 경유 도로를 실제 통행 방향의 최단 경로로 잇는다. 경로가 남긴 도로 밖을
   지나면 거절한다.
4. 배경 교통은 실행망 경계의 진입로마다 정한 유입량을, 되돌아가는 진출로를 뺀 나머지 진출로에
   같은 비율로 나눈 출발·도착 흐름이다. 경로는 SUMO가 고른다.
5. scripts/validate_road.py의 validate_structure로 ego 경로의 차선 연결, 정류장 위치, EGO_ROUTE,
   설계 구역(DESIGN)이 가리키는 도로·교차로·경로가 실행망에 있는지 본다.

검사를 모두 통과하면 두 파일을 같은 폴더의 임시 파일로 먼저 다 쓴 뒤, os.replace 두 번(도로망,
map.json 순서)으로 연달아 바꿔 넣는다. 두 번째 교체가 실패하면 새 도로망과 이전 map.json이 남는다.
도로망이 바뀌었다면 이 짝은 maps.load_map의 sha256 검사가 거절하고, 바뀌지 않았다면 이전 map.json이
그 도로망과 맞는다.

도로 좌표는 여기서 만들지 않는다. 경유 도로를 바꾸면 이 스크립트만 다시 실행한다.
중간 파일은 저장소에 남기지 않는다. 검사가 하나라도 실패하면 저장소의 파일은 그대로다.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.patch_network import FOLDER, build_design_network, digest
from scripts.prune_network import add_osm_ways, prune_network
from scripts.validate_road import validate_structure

VERSION = "6.0.0"
# 학습·평가·수집에서 ego가 달리는 경로. 아래 ROUTES의 이름 중 하나다.
EGO_ROUTE = "r0"
# (설명, 경유 도로). 모두 source/drawn_edges.json의 도로 안에 있어야 한다.
ROUTES = {
    "r0": ("남쪽 정조로 → 중부대로 교차로 직진 → 골목 합류 구간 → 팔달문 남→동→북 → 북쪽 정조로 (ego 기본 경로)",
           ["171323844#2", "37394478#2", "1197050172#0", "1197050172#1", "1197050172#2", "1176921670#3"]),
    "s_alley_merge_north": ("[실험] 동쪽 골목 정조로764번길 → 정조로 북행 2차로에 우회전 합류 → 팔달문 진입 → 북쪽 정조로",
                            ["-59150694#1", "37394478#4", "1197050172#0", "1197050172#2", "1176921670#3"]),
    "s_left_turn": ("[실험] 팔달문 남쪽 출구 → 정조로 남행 2차로 → 중부대로 교차로 좌회전 → 중부대로 동쪽",
                    ["-37394478#7", "171323839", "1224432174#5"]),
    "s_alley_left": ("[실험] 서쪽 골목 → 정조로 남행에 우회전 합류 → 왼쪽 차로 변경 → 중부대로 교차로 좌회전 → 중부대로 동쪽",
                     ["-40197651#1", "-37394478#5", "171323839", "1224432174#5"]),
}
# 경계 진입로의 baseline 유입량(대/시간). 실측값이 아닌 합성 수요다.
# 전체 도로망을 수입할 때 service(건물 진입로·이면도로)라 빠졌지만 팀이 표시한 OSM 길. 설계망에 그대로 붙인다.
ADDED_OSM_WAYS = {
    "59150692": "정조로770번길과 팔달문로 사이 골목",
    "472306783": "팔달문로3번길 동쪽 골목",
    "472306788": "정조로796번길 동쪽 끝",
    "537277616": "팔달문로4번길",
}
ARTERIAL_PER_LANE = 150   # 제한속도 50km/h 도로, 차로당. 교차로 앞에서 넓어진 차로는 2개까지만 센다
ALLEY = 40                # 제한속도 30km/h 골목
# 실제 OSM 정류장 노드와 그 정류장이 놓인 도로. 버스 운행은 도로 범위 밖이므로 위치만 둔다.
BUS_STOPS = {"6210299488": "1197050172#2", "9360442452": "1176921670#2"}
DESIGN = {
    "signal_locations": "source/structure_patch.json의 signal_locations. 실행망에는 신호가 없다.",
    "zones": {
        "roundabout": {"edges": ["1197050172#0", "1197050172#1", "1197050172#2", "38315385#0", "38315385#1", "38315385#2"],
                       "note": "4차로 원형 차도. 진입: 남 정조로 37394478#7, 북 정조로 -1151407004#0, 동 팔달문로 671268554#3, 서 정조로777번길 -1289563568#1, 북동 골목 -59150681#0"},
        "alley_merge_north": {"alley": "-59150694#1", "joins": "37394478#4", "turn": "right",
                              "note": "원형 차도 진입 약 60m 앞에서 정조로 북행 2차로에 합류"},
        "alley_merge_south": {"alley": "-40197651#1", "joins": "-37394478#5", "turn": "right",
                              "note": "합류 후 첫 교차로까지 정조로 남행 약 150m"},
        "left_turn": {"approach": "-37394478#0", "left_lane_index": 1, "node": "414688141", "left_to": "171323839",
                      "goal_edge": "1224432174#5",
                      "note": "좌회전은 왼쪽 차로(index 1)에서만 가능. 서쪽 골목에서 합류하면 교차로까지 약 150m 안에 왼쪽으로 옮겨야 한다"},
    },
}


def shortest_route(net, anchors: list[str], name: str) -> list[str]:
    """경유 도로를 순서대로 실제 통행 방향의 차선 연결을 따라 잇는다."""
    result = []
    for start_id, end_id in zip(anchors, anchors[1:]):
        path, _cost = net.getShortestPath(net.getEdge(start_id), net.getEdge(end_id))
        if path is None:
            raise ValueError(f"{name}: 실제 통행 방향으로 연결되지 않습니다: {start_id} → {end_id}")
        part = [edge.getID() for edge in path]
        result.extend(part if not result else part[1:])
    return result


def boundary_flows(net) -> list[dict]:
    """경계 진입로마다 유입량을 정하고, 되돌아가는 진출로를 뺀 진출로에 같은 비율로 나눈다."""
    boundary = {node.getID() for node in net.getNodes() if node.getType() == "dead_end"}
    entries = [edge for edge in net.getEdges() if edge.getFromNode().getID() in boundary]
    exits = [edge for edge in net.getEdges() if edge.getToNode().getID() in boundary]
    flows = []
    for entry in entries:
        rate = ARTERIAL_PER_LANE * min(entry.getLaneNumber(), 2) if entry.getSpeed() > 10 else ALLEY
        targets = [exit for exit in exits if exit.getToNode() != entry.getFromNode()]
        for exit in targets:
            if net.getShortestPath(entry, exit)[0] is None:
                raise ValueError(f"진입로 {entry.getID()}에서 진출로 {exit.getID()}로 갈 수 없습니다.")
            flows.append({"id": f"f_{entry.getID()}_to_{exit.getID()}", "from": entry.getID(), "to": exit.getID(),
                          "vehs_per_hour": round(rate / len(targets), 3)})
    return flows


def bus_stops(net) -> list[dict]:
    """OSM 정류장 노드를 지정한 도로의 바깥 차로(index 0)에 투영한다."""
    nodes = {node.get("id"): node for node in ET.parse(FOLDER / "source" / "area.osm.xml").getroot().findall("node")}
    stops = []
    for osm_id, edge_id in BUS_STOPS.items():
        tags = {tag.get("k"): tag.get("v") for tag in nodes[osm_id].findall("tag")}
        lon, lat = float(nodes[osm_id].get("lon")), float(nodes[osm_id].get("lat"))
        x, y = net.convertLonLat2XY(lon, lat)
        lane = net.getEdge(edge_id).getLane(0)
        offset, distance = sumolib.geomhelper.polygonOffsetAndDistanceToPoint((x, y), lane.getShape())
        end = min(max(offset + 12.5, 26), lane.getLength() - 1)
        stops.append({"id": f"osm_stop_{osm_id}", "name": tags["name"], "ref": tags.get("ref"),
                      "lane": lane.getID(), "startPos": round(max(0.5, end - 25), 2), "endPos": round(end, 2),
                      "lanes_on_edge": lane.getEdge().getLaneNumber(),
                      "source_osm_node": osm_id, "longitude": lon, "latitude": lat,
                      "snap_distance_m": round(distance, 3)})
    return stops


def publish(files: dict[Path, bytes]) -> None:
    """파일마다 같은 폴더의 임시 파일을 먼저 다 쓰고 권한을 맞춘 뒤, 넣은 순서대로 연달아 바꿔 넣는다."""
    staged = []
    try:
        for path, content in files.items():
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as handle:
                staged.append((Path(handle.name), path))
                handle.write(content)
            staged[-1][0].chmod(0o644)
        for temporary, path in staged:
            os.replace(temporary, path)
    finally:
        for temporary, _ in staged:
            temporary.unlink(missing_ok=True)


def build() -> dict:
    with tempfile.TemporaryDirectory(prefix="paldalmun_build_") as temporary:
        design_path, run_path = Path(temporary) / "network.design.net.xml", Path(temporary) / "network.net.xml"
        structure = build_design_network(design_path)
        added_path = Path(temporary) / "network.added.net.xml"
        service_speed = json.loads((FOLDER / "source" / "speed_defaults.json").read_text(encoding="utf-8"))["highway_defaults"]["service"] / 3.6
        added_edges = add_osm_ways(design_path, added_path, ADDED_OSM_WAYS, FOLDER / "source" / "area.osm.xml", service_speed)
        design_path = added_path
        net = sumolib.net.readNet(str(design_path))
        aliases = structure["edge_aliases"]

        def resolve(anchors):
            missing = [anchor for anchor in anchors if not net.hasEdge(anchor) and anchor not in aliases]
            if missing:
                raise ValueError(f"설계망에 없는 경유 도로입니다: {missing}")
            return [anchor if net.hasEdge(anchor) else aliases[anchor] for anchor in anchors]
        drawn = json.loads((FOLDER / "source" / "drawn_edges.json").read_text(encoding="utf-8"))["edges"]
        prune_network(design_path, run_path, drawn)
        run_net = sumolib.net.readNet(str(run_path))
        routes, route_design = {}, {}

        def facts(description, route):
            # 실행 진입점이 netconvert 없이 읽는 경로 사실. 길이·제한속도·출발 구간 차로 수.
            return {"description": description,
                    "length_m": round(sum(net.getEdge(e).getLength() for e in route), 2),
                    "max_speed_mps": max(lane.getSpeed() for e in route for lane in net.getEdge(e).getLanes()),
                    "first_edge_lanes": net.getEdge(route[0]).getLaneNumber()}
        for route_id, (description, anchors) in ROUTES.items():
            routes[route_id] = shortest_route(run_net, resolve(anchors), route_id)
            route_design[route_id] = facts(description, routes[route_id])
        flows = boundary_flows(run_net)
        center = net.convertLonLat2XY(127.01700, 37.27660)
        definition = {
            "id": "paldalmun", "name": "팔달문 · 원형 차도와 주변 골목", "version": VERSION,
            "description": "팔달문 원형 차도를 중심으로 정조로, 행궁로, 수원천로 양안, 중부대로와 그 사이 골목. 팀이 지도에 표시한 도로만 남겼고 신호는 없다.",
            "source_status": "georeferenced_osm",
            "network_source": {
                "path": "source/network.net.xml", "sha256": digest(run_path),
                "full_path": "source/network.full.net.xml", "full_sha256": digest(FOLDER / "source" / "network.full.net.xml"),
                "design_sha256": structure["sha256"], "netconvert_version": structure["netconvert_version"],
                "structure_patch": "source/structure_patch.json",
                "added_osm_ways": {"ways": ADDED_OSM_WAYS, "edges": added_edges, "speed_mps": round(service_speed, 3),
                                   "rule": "OSM 좌표 그대로, 1차로 양방향, 제한속도는 source/speed_defaults.json의 service"},
                **{key: structure[key] for key in ("edge_aliases", "removed_signal_nodes", "removed_tls_programs",
                                                   "dissolved_geometry_nodes", "lane_end_retrim_m",
                                                   "original_point_max_deviation_m")},
            },
            "focus_bbox": [127.0141, 37.2728, 127.0204, 37.2812],
            "gui_center": {"x": round(center[0], 3), "y": round(center[1], 3)},
            "sources_license": {"road_data": "ODbL 1.0", "attribution": "© OpenStreetMap contributors",
                                "url": "https://www.openstreetmap.org/copyright"},
            "references": [{"title": "실제 도로 좌표·곡선·통행 방향 원본", "source": "source/area.osm.xml",
                            "url": "https://www.openstreetmap.org/", "license": "ODbL 1.0"}],
            "assumptions": [
                "교차로 노드는 OSM 좌표에서 1cm 안, 원형 차도 밖의 도로 중심선은 OSM 도로선에서 1mm 안이다. 원형 차도는 OSM 곡선점 사이를 원호로 보간해 OSM 직선 연결과 최대 0.59m 다르다.",
                "차로 수가 OSM에 없던 구간은 SUMO 기본값이다. 정조로 횡단면은 source/overrides.json, 팔달문.이안과 정류장 구간의 1→2차로 확장은 source/simulation_overrides.json, 누락 제한속도는 source/speed_defaults.json의 가정이다.",
                "실행망에는 신호가 없다. 실제 신호 위치는 source/structure_patch.json의 signal_locations에 있다.",
                "유입량은 실측값이 아닌 도로 검증용 합성 수요다. quiet·baseline·busy는 0.5·1·1.25배다.",
            ],
            "routes": routes, "route_design": route_design, "flows": flows,
            "max_lane_speed_mps": max(lane.getSpeed() for edge in run_net.getEdges() for lane in edge.getLanes()),
            "ego_route": EGO_ROUTE,
            "bus_stops": bus_stops(run_net),
            "design": DESIGN,
        }
        validate_structure(definition, run_path)
        publish({FOLDER / "source" / "network.net.xml": run_path.read_bytes(),
                 FOLDER / "map.json": (json.dumps(definition, ensure_ascii=False, indent=2) + "\n").encode("utf-8")})
    return definition


if __name__ == "__main__":
    result = build()
    edges = sum(edge.get("function") != "internal"
                for edge in ET.parse(FOLDER / "source" / "network.net.xml").getroot().findall("edge"))
    print(f"paldalmun {result['version']}: 경로 {len(result['routes'])}개, 실행 도로 {edges}개, "
          f"정류장 {len(result['bus_stops'])}곳, 실행망 sha256 {result['network_source']['sha256'][:12]}")
