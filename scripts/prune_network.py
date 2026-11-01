"""정한 도로만 남긴 실행망을 만든다.

scripts/build_road.py가 prune_network(design, output, edges)로 호출한다.

netconvert의 --keep-edges.explicit로 나머지 도로를 지우고, 남은 도로만으로 교차로 모양과
통행 우선순위를 다시 계산한다. 그래서 남기지 않은 옆길의 자투리가 생기지 않는다. 다시 계산해도
도로의 원래 형상(교차로 중심까지 잇는 선)과 교차로 좌표는 설계망과 같아야 하며, 다르면 거절한다.
"""
from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Iterable
import xml.etree.ElementTree as ET

import sumolib

from scripts.patch_network import GEOMETRY_PRESERVING_OPTIONS

# 지운 옆길 때문에 2갈래가 된 교차로를 합치면 도로 id가 바뀌므로 합치지 않는다.
OPTIONS = list(GEOMETRY_PRESERVING_OPTIONS)
OPTIONS[OPTIONS.index("--geometry.remove") + 1] = "false"


def prune_network(design_path: Path | str, output_path: Path | str, edges: Iterable[str]) -> dict:
    design_path, output_path = Path(design_path), Path(output_path)
    if design_path.resolve() == output_path.resolve():
        raise ValueError("설계망과 실행망 출력 경로는 달라야 합니다.")
    keep = sorted(set(edges))
    completed = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(design_path),
                                "--output-file", str(output_path), *OPTIONS, "--keep-edges.explicit", ",".join(keep)],
                               capture_output=True, text=True, timeout=120)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    # netconvert는 생성 시각과 임시 폴더 경로를 머리 주석에 쓴다. 다시 쓰면 주석이 빠져 빌드마다 같은 바이트가 된다.
    ET.ElementTree(ET.parse(output_path).getroot()).write(output_path, encoding="utf-8", xml_declaration=True)
    design, run = sumolib.net.readNet(str(design_path)), sumolib.net.readNet(str(output_path))
    if sorted(edge.getID() for edge in run.getEdges()) != keep:
        raise ValueError("실행망 도로가 남기기로 한 도로와 다릅니다.")
    moved = [edge.getID() for edge in run.getEdges() if edge.getRawShape() != design.getEdge(edge.getID()).getRawShape()]
    moved += [node.getID() for node in run.getNodes() if node.getCoord() != design.getNode(node.getID()).getCoord()]
    if moved:
        raise ValueError(f"가지치기가 도로 형상이나 교차로 좌표를 바꿨습니다: {moved}")
    return {"edges": len(keep), "junctions": len(run.getNodes())}


def add_osm_ways(design_path: Path | str, output_path: Path | str, way_ids: Iterable[str], osm_path: Path | str,
                 speed_mps: float) -> list[str]:
    """설계망에 없는 OSM 길을 OSM 좌표 그대로 1차로 양방향 도로로 붙인다.

    길은 설계망 교차로 노드에서 끊어 구간을 만든다. 양 끝은 설계망 교차로여야 한다. 기존 도로의 연결은
    도로망 파일에서 읽으면 고정값이라, 새 구간이 닿는 교차로의 연결을 차로까지 적어 준다(우회전은
    바깥 차로, 그 밖은 안쪽 차로). 기존 도로의 형상이 바뀌면 거절한다. 만든 구간 id를 돌려준다.
    """
    design = sumolib.net.readNet(str(design_path))
    osm = ET.parse(osm_path).getroot()
    lonlat = {node.get("id"): (float(node.get("lon")), float(node.get("lat"))) for node in osm.iter("node")}
    ways = {way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in osm.iter("way")}
    new = {}  # 구간 id → (시작 노드, 끝 노드, 형상)
    for way_id in way_ids:
        nodes = ways[way_id]
        cuts = [index for index, node in enumerate(nodes) if design.hasNode(node)]
        if not cuts or cuts[0] != 0 or cuts[-1] != len(nodes) - 1:
            raise ValueError(f"OSM 길 {way_id}의 양 끝이 설계망 교차로가 아닙니다.")
        for part, (start, end) in enumerate(zip(cuts, cuts[1:])):
            shape = ([design.getNode(nodes[start]).getCoord()]
                     + [design.convertLonLat2XY(*lonlat[node]) for node in nodes[start + 1:end]]
                     + [design.getNode(nodes[end]).getCoord()])
            new[f"{way_id}#{part}"] = (nodes[start], nodes[end], shape)
            new[f"-{way_id}#{part}"] = (nodes[end], nodes[start], shape[::-1])
    edges = ET.Element("edges")
    for edge_id, (start, end, shape) in new.items():
        ET.SubElement(edges, "edge", {"id": edge_id, "from": start, "to": end, "numLanes": "1", "priority": "1",
                                      "speed": f"{speed_mps:.3f}", "spreadType": "center",
                                      "shape": " ".join(f"{x:.3f},{y:.3f}" for x, y in shape)})

    def opposite(edge_id):
        return edge_id[1:] if edge_id.startswith("-") else "-" + edge_id
    connections = ET.Element("connections")
    for node_id in {node for start, end, _ in new.values() for node in (start, end)}:
        node = design.getNode(node_id)
        incoming = [edge.getID() for edge in node.getIncoming()] + [k for k, (_, end, _) in new.items() if end == node_id]
        outgoing = [edge.getID() for edge in node.getOutgoing()] + [k for k, (start, _, _) in new.items() if start == node_id]
        for source in incoming:
            for target in outgoing:
                if (source not in new and target not in new) or target == opposite(source):
                    continue
                attributes = {"from": source, "to": target}
                if source not in new:
                    edge = design.getEdge(source)
                    (x0, y0), (x1, y1) = edge.getShape()[-2:]
                    x2, y2 = new[target][2][1]
                    right_turn = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1) < 0
                    attributes.update(fromLane="0" if right_turn else str(edge.getLaneNumber() - 1), toLane="0")
                ET.SubElement(connections, "connection", attributes)
    folder = Path(output_path).parent
    ET.ElementTree(edges).write(folder / "added.edg.xml", encoding="utf-8", xml_declaration=True)
    ET.ElementTree(connections).write(folder / "added.con.xml", encoding="utf-8", xml_declaration=True)
    completed = subprocess.run([sumolib.checkBinary("netconvert"), "--sumo-net-file", str(design_path),
                                "--edge-files", str(folder / "added.edg.xml"), "--connection-files",
                                str(folder / "added.con.xml"), "--output-file", str(output_path), *OPTIONS],
                               capture_output=True, text=True, timeout=120)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    result = sumolib.net.readNet(str(output_path))
    moved = [edge.getID() for edge in design.getEdges() if edge.getRawShape() != result.getEdge(edge.getID()).getRawShape()]
    unconnected = [edge_id for edge_id in new if not result.getEdge(edge_id).getIncoming()
                   and result.getEdge(edge_id).getFromNode().getIncoming()]
    if moved or unconnected:
        raise ValueError(f"OSM 길을 붙이다 기존 형상이 바뀌었거나 들어오는 연결이 없습니다: {moved} {unconnected}")
    return sorted(new)
