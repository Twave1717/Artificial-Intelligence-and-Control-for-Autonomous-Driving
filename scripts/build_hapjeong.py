"""합정역 map.json을 만든다.

    python scripts/build_hapjeong.py

합정역 도로망(maps/hapjeong/source/network.net.xml)은 이 스크립트가 만들지 않는다. 서예인이
maps/hapjeong/generator/의 생성기로 만든 결과를 그대로 쓴다. 이 스크립트는 그 도로망에서
ego 경로의 길이·제한속도, 경계 진입로의 배경 흐름(팔달문과 같은 규칙), 화면 중심을 계산하고
scripts/validate_road.py의 validate_structure로 확인한 뒤 map.json만 쓴다.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import sumolib

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from scripts.build_road import boundary_flows, publish
from scripts.patch_network import digest
from scripts.validate_road import validate_structure

FOLDER = BASE / "maps" / "hapjeong"
VERSION = "1.0.0"
EGO_ROUTE = "r_sangsu_to_yanghwa"
ROUTES = {EGO_ROUTE: ("상수동 쪽 → 합정역 교차로 → 양화로", ["R111", "R112", "R113", "R100", "-R96", "R95", "R94"])}
EXTRA_FILES = {
    "additional": ["source/signals.add.xml", "source/islands.add.xml", "source/seams.add.xml", "source/stops.add.xml"],
    "routes": ["source/pedestrians.rou.xml"],
}


def build() -> dict:
    net_path = FOLDER / "source" / "network.net.xml"
    net = sumolib.net.readNet(str(net_path))
    routes, route_design = {}, {}
    for route_id, (description, edges) in ROUTES.items():
        routes[route_id] = edges
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
        "id": "hapjeong", "name": "합정역 · 양화로와 독막로 골목", "version": VERSION,
        "description": "합정역 일대 실제 도로. 신호 14곳, 인도·횡단보도·보행자, 버스 전용 차로가 있다.",
        "source_status": "osm_generated",
        "network_source": {"path": "source/network.net.xml", "sha256": digest(net_path),
                           "generator": "generator/ (서예인, 합정역 브랜치 ec6fbf6에서 옮김)"},
        "extra_files": EXTRA_FILES,
        "trainable": False,
        "trainable_reason": "일반 도로 95개 중 79개의 0번 차로가 인도라서, 차로 번호를 관측으로 쓰는 학습 환경이 틀린 값을 읽는다.",
        "focus_bbox": [round(west, 7), round(south, 7), round(east, 7), round(north, 7)],
        "gui_center": {"x": round(center[0], 3), "y": round(center[1], 3)},
        "sources_license": {"road_data": "ODbL 1.0", "attribution": "© OpenStreetMap contributors",
                            "url": "https://www.openstreetmap.org/copyright"},
        "assumptions": ["배경 차량 유입량은 팔달문과 같은 규칙의 합성 수요다. quiet·baseline·busy는 0.5·1·1.25배다.",
                        "보행자(source/pedestrians.rou.xml)는 시나리오와 관계없이 같은 수다."],
        "routes": routes, "route_design": route_design, "flows": boundary_flows(net),
        "max_lane_speed_mps": max(lane.getSpeed() for edge in net.getEdges() for lane in edge.getLanes()
                                  if lane.allows("passenger")),
        "ego_route": EGO_ROUTE, "bus_stops": [],
        "design": {"measure_lanes": [], "zones": {}},
    }
    validate_structure(definition, net_path)
    publish({FOLDER / "map.json": (json.dumps(definition, ensure_ascii=False, indent=2) + "\n").encode("utf-8")})
    return definition


if __name__ == "__main__":
    result = build()
    print(f"hapjeong {result['version']}: 배경 흐름 {len(result['flows'])}개, "
          f"baseline {round(sum(flow['vehs_per_hour'] for flow in result['flows']))}대/시간, "
          f"실행망 sha256 {result['network_source']['sha256'][:12]}")
