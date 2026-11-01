"""OSM 수입망에서 신호를 걷어내고 겹친 분할점만 합친 설계망을 만든다.

    python scripts/patch_network.py

maps/paldalmun/source/structure_patch.json에 따라 network.full.net.xml을 읽어 설계망을
호출자가 준 경로(보통 임시 폴더)에 만든다. 원본 OSM을 다시 변환하지 않으므로 PROJ가 없는
netconvert로도 실행할 수 있다. 원래 좌표가 1cm 넘게 움직이거나 허용하지 않은 도로·진행
방향이 바뀌면 예외를 낸다. scripts/build_road.py가 이 설계망을 경로 탐색과 가지치기의
기준으로 쓴다. 설계망은 저장소에 두지 않고 매번 다시 만든다.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from sumolib import checkBinary

FOLDER = Path(__file__).resolve().parents[1] / "maps" / "paldalmun"

# 원본 수입 때와 같은 형상 보존 옵션이다. 분할점 병합만 geometry.remove를 켠다.
GEOMETRY_PRESERVING_OPTIONS = [
    "--geometry.remove", "true",
    "--geometry.min-dist", "0",
    "--geometry.max-angle.fix", "false",
    "--geometry.min-radius.fix", "false",
    "--geometry.avoid-overlap", "false",
    "--junctions.join", "false",
    "--roundabouts.guess", "false",
    "--no-turnarounds", "true",
    "--output.original-names", "true",
    "--output.street-names", "true",
    "--junctions.corner-detail", "12",
    "--junctions.internal-link-detail", "12",
    "--precision", "3", "--precision.geo", "9",
]
# 합친 도로가 원래 OSM 점을 지나는지 볼 때의 허용 오차(m). XML 반올림만 허용한다.
POINT_TOLERANCE_M = 0.01
# 외곽선을 다시 계산한 교차로 쪽 차선 끝점만 움직일 수 있다. 중심선이 아닌 정지 위치다.
RETRIM_LIMIT_M = 0.5


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def signature(element: ET.Element) -> tuple:
    """들여쓰기만 무시하고 모든 속성·자식 정보를 비교한다."""
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(signature(child) for child in element))


def points(text: str) -> list[tuple[float, float]]:
    return [tuple(map(float, point.split(",")[:2])) for point in text.split()]


def distance_to_polyline(point, polyline) -> float:
    best = math.inf
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        dx, dy = bx - ax, by - ay
        length = dx * dx + dy * dy
        t = 0.0 if length == 0 else max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / length))
        best = min(best, math.dist(point, (ax + t * dx, ay + t * dy)))
    return best


def owner(internal_id: str) -> str:
    return internal_id[1:].rsplit("_", 1)[0]


def external_edges(root: ET.Element) -> dict[str, ET.Element]:
    return {edge.get("id"): edge for edge in root.findall("edge") if edge.get("function") != "internal"}


def junctions(root: ET.Element) -> dict[str, ET.Element]:
    return {node.get("id"): node for node in root.findall("junction") if node.get("type") != "internal"}


def signal_nodes(root: ET.Element) -> set[str]:
    return {key for key, node in junctions(root).items() if node.get("type", "").startswith("traffic_light")}


def centerline(edge: ET.Element, nodes: dict[str, ET.Element]) -> list[tuple[float, float]]:
    if edge.get("shape"):
        return points(edge.get("shape"))
    return [(float(nodes[edge.get(end)].get("x")), float(nodes[edge.get(end)].get("y"))) for end in ("from", "to")]


def geometry_neighbors(full: ET.Element, geometry: set[str]) -> dict[str, set[str]]:
    """합칠 노드가 여전히 옆길 없는 양방향 분할점인지 원본에서 확인한다."""
    edges = external_edges(full)
    neighbors = {}
    for node_id in geometry:
        incoming = [key for key, edge in edges.items() if edge.get("to") == node_id]
        outgoing = [key for key, edge in edges.items() if edge.get("from") == node_id]
        around = {edges[key].get("from") for key in incoming} | {edges[key].get("to") for key in outgoing}
        if len(incoming) != 2 or len(outgoing) != 2 or len(around) != 2:
            raise ValueError(f"{node_id}에 옆길·분기가 있어 분할점으로 합칠 수 없습니다.")
        neighbors[node_id] = around
    return neighbors


def merged_aliases(full: ET.Element, design: ET.Element, geometry: set[str]) -> tuple[dict[str, str], float]:
    """합쳐진 도로 ID를 남은 도로 ID로 잇고, 원래 중심선 점이 그대로 지나가는지 확인한다.

    양방향 도로는 두 방향이 같은 중심선을 공유하므로 진행 방향이 같은 도로만 받는다.
    원래 점이 합친 도로에서 벗어난 최대 거리(m)를 같이 준다.
    """
    full_edges, design_edges = external_edges(full), external_edges(design)
    full_nodes, design_nodes = junctions(full), junctions(design)
    incident = {key for key, edge in full_edges.items() if edge.get("from") in geometry or edge.get("to") in geometry}
    survivors = {key: centerline(design_edges[key], design_nodes) for key in incident if key in design_edges}

    def heading(line):
        return line[-1][0] - line[0][0], line[-1][1] - line[0][1]
    aliases, deviation = {}, 0.0
    for key in sorted(incident):
        original = centerline(full_edges[key], full_nodes)
        dx, dy = heading(original)
        hosts = [survivor for survivor, line in survivors.items()
                 if all(distance_to_polyline(point, line) <= POINT_TOLERANCE_M for point in original)
                 and dx * heading(line)[0] + dy * heading(line)[1] > 0]
        if len(hosts) != 1:
            raise ValueError(f"합친 도로 {key}의 원래 중심선을 지나는 같은 방향 도로를 하나로 정할 수 없습니다: {hosts}")
        deviation = max(deviation, *(distance_to_polyline(point, survivors[hosts[0]]) for point in original))
        if hosts[0] != key:
            aliases[key] = hosts[0]
    for node_id in geometry:
        position = (float(full_nodes[node_id].get("x")), float(full_nodes[node_id].get("y")))
        through = {key: distance_to_polyline(position, line) for key, line in survivors.items()}
        through = {key: value for key, value in through.items() if value <= POINT_TOLERANCE_M}
        if len(through) != 2:
            raise ValueError(f"분할점 {node_id}의 원래 좌표를 양방향 도로가 모두 지나지 않습니다: {sorted(through)}")
        deviation = max(deviation, *through.values())
    return aliases, deviation


def lane_end_retrim(before: ET.Element, after: ET.Element, recomputed: set[str]) -> float | None:
    """외곽선을 다시 계산한 교차로에 닿은 차선 끝점만 움직였으면 그 거리를, 아니면 None을 준다."""
    if before.attrib != after.attrib or len(before.findall("lane")) != len(after.findall("lane")):
        return None
    movable = [index for end, index in (("from", 0), ("to", -1)) if before.get(end) in recomputed]
    shift = 0.0
    for old, new in zip(before.findall("lane"), after.findall("lane")):
        if {k: v for k, v in old.attrib.items() if k not in ("shape", "length")} != \
                {k: v for k, v in new.attrib.items() if k not in ("shape", "length")}:
            return None
        a, b = points(old.get("shape")), points(new.get("shape"))
        if len(a) != len(b):
            return None
        for index, (p, q) in enumerate(zip(a, b)):
            moved = math.dist(p, q)
            if moved == 0:
                continue
            if index - len(a) not in movable and index not in movable or moved > RETRIM_LIMIT_M:
                return None
            shift = max(shift, moved)
    return round(shift, 3)


def verify_design(full: ET.Element, design: ET.Element, geometry: set[str]) -> dict:
    """신호 제거와 분할점 병합 외에는 바뀐 것이 없음을 원본과 직접 대조한다."""
    if design.findall("tlLogic") or signal_nodes(design):
        raise ValueError("설계망에 신호가 남아 있습니다.")
    neighbors = geometry_neighbors(full, geometry)
    around = set().union(*neighbors.values()) if neighbors else set()
    full_external, design_external = external_edges(full), external_edges(design)
    incident = {key for key, edge in full_external.items() if edge.get("from") in geometry or edge.get("to") in geometry}
    if set(design_external) - set(full_external) or (set(full_external) - set(design_external)) - incident:
        raise ValueError("분할점 밖에서 일반 도로가 생기거나 사라졌습니다.")
    retrim = {}
    for key in sorted(set(design_external) - incident):
        before, after = full_external[key], design_external[key]
        if signature(before) == signature(after):
            continue
        shift = lane_end_retrim(before, after, around)
        if shift is None:
            raise ValueError(f"일반 도로 구간의 형상·차로 속성이 바뀌었습니다: {key}")
        retrim[key] = shift
    aliases, deviation = merged_aliases(full, design, geometry)
    touched = signal_nodes(full) | geometry | around

    def internal(root):
        return {edge.get("id"): edge for edge in root.findall("edge") if edge.get("function") == "internal"}
    full_internal, design_internal = internal(full), internal(design)
    for key in set(full_internal) | set(design_internal):
        if owner(key) not in touched and (key not in full_internal or key not in design_internal
                                          or signature(full_internal[key]) != signature(design_internal[key])):
            raise ValueError(f"보정 대상이 아닌 교차로의 내부 도로가 바뀌었습니다: {key}")
    full_nodes, design_nodes = junctions(full), junctions(design)
    if set(full_nodes) - set(design_nodes) != geometry or set(design_nodes) - set(full_nodes):
        raise ValueError("합친 분할점 밖에서 교차로가 생기거나 사라졌습니다.")
    for key, node in design_nodes.items():
        if any(node.get(attribute) != full_nodes[key].get(attribute) for attribute in ("x", "y", "z")):
            raise ValueError(f"교차로 좌표가 바뀌었습니다: {key}")
        if key not in around and node.get("shape") != full_nodes[key].get("shape"):
            raise ValueError(f"분할점과 닿지 않은 교차로의 외곽선이 바뀌었습니다: {key}")
        if key not in touched and signature(node) != signature(full_nodes[key]):
            raise ValueError(f"보정 대상이 아닌 교차로의 우선순위 규칙이 바뀌었습니다: {key}")

    def movements(root, edges, rename):
        # 합친 분할점 위의 직진 연결은 한 도로 안으로 들어가므로 비교에서 뺀다.
        return {(rename.get(c.get("from"), c.get("from")), rename.get(c.get("to"), c.get("to")),
                 c.get("fromLane"), c.get("toLane"), c.get("dir"))
                for c in root.findall("connection")
                if not c.get("from", "").startswith(":") and edges[c.get("from")].get("to") not in geometry}
    if movements(full, full_external, aliases) != movements(design, design_external, {}):
        raise ValueError("차로 단위 진행 방향이 바뀌었습니다.")
    return {
        "edge_aliases": aliases,
        "removed_signal_nodes": len(signal_nodes(full)),
        "removed_tls_programs": len(full.findall("tlLogic")),
        "dissolved_geometry_nodes": sorted(geometry),
        "lane_end_retrim_m": retrim,
        "original_point_max_deviation_m": round(deviation, 4),
    }


def build_design_network(output: Path) -> dict:
    """full 원본에서 설계망을 output에 만들고 원본과 대조한 결과를 준다. 같은 입력이면 같은 설계망이 나온다."""
    netconvert = checkBinary("netconvert")
    patch = json.loads((FOLDER / "source" / "structure_patch.json").read_text(encoding="utf-8"))
    source = FOLDER / patch["applies_to"]
    full = ET.parse(source).getroot()
    geometry = {item["node"] for item in patch["geometry_nodes"]}
    geometry_neighbors(full, geometry)
    node_root = ET.Element("nodes")
    # --tls.unset은 프로그램만 지우고 노드를 traffic_light로 남기므로 형식을 직접 바꾼다.
    for node in sorted(signal_nodes(full)):
        ET.SubElement(node_root, "node", id=node, type="priority")
    with tempfile.TemporaryDirectory(prefix="design_network_") as temporary:
        node_file, keep_file = Path(temporary) / "signals.nod.xml", Path(temporary) / "keep-edges.txt"
        ET.ElementTree(node_root).write(node_file, encoding="utf-8", xml_declaration=True)
        keep_file.write_text("\n".join(sorted(key for key, edge in external_edges(full).items()
                                              if edge.get("from") not in geometry and edge.get("to") not in geometry)) + "\n",
                             encoding="utf-8")
        command = [netconvert, "--sumo-net-file", str(source), "--output-file", str(output),
                   *GEOMETRY_PRESERVING_OPTIONS, "--node-files", str(node_file),
                   "--geometry.remove.keep-edges.input-file", str(keep_file)]
        completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode:
        raise RuntimeError("netconvert 실패: " + completed.stderr[-3000:])
    tree = ET.parse(output)
    location, original = tree.getroot().find("location"), full.find("location")
    # PROJ 없는 netconvert는 origBoundary를 소수 셋째 자리로 줄여 쓴다. 투영식·오프셋·범위가
    # 그대로인지 확인한 뒤 원본의 location 요소를 다시 넣는다.
    for key in ("netOffset", "projParameter", "convBoundary"):
        if location.get(key) != original.get(key):
            raise ValueError(f"netconvert가 좌표 변환 {key}를 바꿨습니다: {location.get(key)!r} != {original.get(key)!r}")
    location.attrib.clear()
    location.attrib.update(original.attrib)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    verification = verify_design(full, ET.parse(output).getroot(), geometry)
    version = subprocess.run([netconvert, "--version"], capture_output=True, text=True, check=True).stdout.splitlines()[0]
    return {"netconvert_version": version, "sha256": digest(output), **verification}


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="design_network_") as temporary:
        result = build_design_network(Path(temporary) / "network.design.net.xml")
    print(f"신호 노드 {result['removed_signal_nodes']}곳 제거, 분할점 {len(result['dissolved_geometry_nodes'])}곳 병합, "
          f"원래 점 최대 이탈 {result['original_point_max_deviation_m']}m, "
          f"차선 끝점 최대 이동 {max(result['lane_end_retrim_m'].values(), default=0)}m")
