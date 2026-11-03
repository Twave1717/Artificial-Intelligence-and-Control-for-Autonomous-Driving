"""실제 지도(OpenStreetMap) 좌표 → SUMO 도로망 생성기.

구글맵 등에서 찍은 위도/경도와 반경(m)을 주면
    1) Overpass API로 해당 범위의 OSM 데이터(도로/건물/물/공원)를 받고
    2) 중심에서 반경 밖의 노드를 잘라낸 뒤 (긴 도로가 밖으로 삐져나오지 않게)
    3) netconvert / polyconvert 로 SUMO 도로망(.net.xml)과 배경 폴리곤(.poly.xml)을 만들고
    4) randomTrips.py 로 배경 교통(.rou.xml)과 실행 설정(.sumocfg)까지 만들어준다.

사용 예:
    python env/osm_builder.py --lat 37.5493558 --lon 126.9136561 --radius 1000 --name hapjeong
    sumo-gui -c env/sumo_osm/hapjeong.sumocfg

--to-config 모드:
    OSM의 "큰 도로"(motorway~tertiary 등급과 그 램프)만 골라 단순화한 뒤,
    노드/도로 목록을 env/road_config.py 의 OSM_ROAD 블록으로 써 넣는다.
    이후에는 road_config.py 에서 차선 수/제한속도/좌표를 직접 고치고
    ROAD["layout"] = "osm" 으로 두면 road_builder.py 가 그 값으로 도로를 만든다.

    python env/osm_builder.py --lat 37.5493558 --lon 126.9136561 --radius 700 --name hapjeong --to-config
    python view_road.py
"""

import argparse
import math
import os
import subprocess
import sys
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

from sumolib import checkBinary

SUMO_HOME = os.environ["SUMO_HOME"]
TYPEMAP = os.path.join(SUMO_HOME, "data", "typemap")
OVERPASS_URL = "https://overpass-api.de/api/interpreter"
EARTH_M_PER_DEG = 111320.0


def bbox(lat: float, lon: float, radius: float):
    """중심/반경 → (south, west, north, east)"""
    dlat = radius / EARTH_M_PER_DEG
    dlon = radius / (EARTH_M_PER_DEG * math.cos(math.radians(lat)))
    return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def download(lat: float, lon: float, radius: float, path: str):
    s, w, n, e = bbox(lat, lon, radius)
    b = f"({s:.6f},{w:.6f},{n:.6f},{e:.6f})"
    query = (f'[out:xml][timeout:180];('
             f'way["highway"]{b};way["railway"]{b};way["building"]{b};'
             f'way["natural"="water"]{b};way["waterway"="riverbank"]{b};'
             f'way["leisure"="park"]{b};way["landuse"]{b};'
             f');(._;>;);out body;')
    req = urllib.request.Request(
        OVERPASS_URL,
        data=urllib.parse.urlencode({"data": query}).encode(),
        headers={"User-Agent": "sumo-osm-builder/1.0"})
    with urllib.request.urlopen(req, timeout=240) as r, open(path, "wb") as f:
        f.write(r.read())


def clip(src: str, dst: str, lat: float, lon: float, radius: float):
    """반경 밖 노드를 제거하고, way는 '연속으로 안쪽에 있는 노드 구간'별로 쪼갠다.
    도로(highway/railway)만 쪼개고, 폴리곤(건물 등)은 노드가 하나라도 밖이면 버린다."""
    tree = ET.parse(src)
    root = tree.getroot()
    kx = EARTH_M_PER_DEG * math.cos(math.radians(lat))
    inside = {}
    for nd in root.findall("node"):
        dx = (float(nd.get("lon")) - lon) * kx
        dy = (float(nd.get("lat")) - lat) * EARTH_M_PER_DEG
        inside[nd.get("id")] = dx * dx + dy * dy <= radius * radius

    next_id = -1
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        tags = way.findall("tag")
        is_line = any(t.get("k") in ("highway", "railway") for t in tags)
        if all(inside.get(r, False) for r in refs):
            continue
        root.remove(way)
        if not is_line:
            continue
        runs, cur = [], []
        for r in refs:
            if inside.get(r, False):
                cur.append(r)
            else:
                if len(cur) >= 2:
                    runs.append(cur)
                cur = []
        if len(cur) >= 2:
            runs.append(cur)
        for run in runs:
            new = ET.SubElement(root, "way", id=str(next_id))
            next_id -= 1
            for r in run:
                ET.SubElement(new, "nd", ref=r)
            for t in tags:
                ET.SubElement(new, "tag", k=t.get("k"), v=t.get("v"))

    for nd in root.findall("node"):
        if not inside[nd.get("id")]:
            root.remove(nd)
    tree.write(dst, encoding="utf-8", xml_declaration=True)


def build(lat: float, lon: float, radius: float, name: str, out_dir: str,
          period: float = 2.0, end: int = 3600, redownload: bool = False) -> str:
    os.makedirs(out_dir, exist_ok=True)
    p = lambda ext: os.path.join(out_dir, f"{name}.{ext}")

    if redownload or not os.path.exists(p("raw.osm.xml")):
        print("[1/5] OSM 다운로드...")
        download(lat, lon, radius, p("raw.osm.xml"))
    print("[2/5] 반경 밖 잘라내기...")
    clip(p("raw.osm.xml"), p("osm.xml"), lat, lon, radius)

    print("[3/5] netconvert...")
    subprocess.run(
        [checkBinary("netconvert"),
         "--osm-files", p("osm.xml"), "-o", p("net.xml"),
         "--type-files", os.path.join(TYPEMAP, "osmNetconvert.typ.xml"),
         "--geometry.remove", "--roundabouts.guess", "--ramps.guess",
         "--junctions.join", "--tls.guess-signals", "--tls.discard-simple", "--tls.join",
         "--keep-edges.by-vclass", "passenger",
         "--remove-edges.isolated", "--keep-edges.components", "1",
         "--output.street-names", "--osm.lane-access", "false",
         "--no-turnarounds.except-deadend", "--no-warnings"],
        check=True)

    print("[4/5] polyconvert (건물/물/공원)...")
    subprocess.run(
        [checkBinary("polyconvert"),
         "--osm-files", p("osm.xml"), "--net-file", p("net.xml"),
         "--type-file", os.path.join(TYPEMAP, "osmPolyconvert.typ.xml"),
         "-o", p("poly.xml")],
        check=True)

    print("[5/5] 배경 교통 생성 (randomTrips)...")
    subprocess.run(
        [sys.executable, os.path.join(SUMO_HOME, "tools", "randomTrips.py"),
         "-n", p("net.xml"), "-r", p("rou.xml"), "-o", p("trips.xml"),
         "-e", str(end), "-p", str(period), "--fringe-factor", "5",
         "--min-distance", "300", "--validate", "--seed", "42"],
        check=True)

    with open(p("sumocfg"), "w", encoding="utf-8") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{name}.net.xml"/>
        <route-files value="{name}.rou.xml"/>
        <additional-files value="{name}.poly.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{end}"/>
        <step-length value="0.5"/>
    </time>
    <processing>
        <time-to-teleport value="120"/>
        <ignore-junction-blocker value="30"/>
    </processing>
</configuration>
""")
    print("완료:", p("sumocfg"))
    return p("sumocfg")


# ──────────────────────────────────────────────────────────────
# --to-config: OSM → road_config.py 의 OSM_ROAD 블록
# ──────────────────────────────────────────────────────────────
# config로 가져올 도로 등급 (OSM highway=*). 골목(residential/service)은 제외.
MAJOR_TYPES = ["motorway", "motorway_link", "trunk", "trunk_link",
               "primary", "primary_link", "secondary", "secondary_link",
               "tertiary", "tertiary_link"]
BLOCK_BEGIN = "# >>> OSM_ROAD >>>"
BLOCK_END = "# <<< OSM_ROAD <<<"


def tmerc(lat: float, lon: float) -> str:
    """중심 좌표가 (0,0)이 되는 횡메르카토르 투영 (단위 m)"""
    return (f"+proj=tmerc +lat_0={lat} +lon_0={lon} +k=1 +x_0=0 +y_0=0 "
            f"+ellps=WGS84 +units=m +no_defs")


def _simplify(pts, tol):
    """Ramer–Douglas–Peucker: 직선에서 tol(m) 이내로 벗어나는 중간점은 버린다."""
    if len(pts) < 3:
        return list(pts)
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    dx, dy = x2 - x1, y2 - y1
    norm = math.hypot(dx, dy) or 1e-9
    dists = [abs(dy * (x - x1) - dx * (y - y1)) / norm for x, y in pts[1:-1]]
    i = max(range(len(dists)), key=dists.__getitem__)
    if dists[i] <= tol:
        return [pts[0], pts[-1]]
    return _simplify(pts[:i + 2], tol)[:-1] + _simplify(pts[i + 1:], tol)


def _shape(s: str):
    return [tuple(map(float, p.split(","))) for p in s.split()]


def export_config(lat: float, lon: float, radius: float, name: str, out_dir: str,
                  config_path: str, redownload: bool = False):
    os.makedirs(out_dir, exist_ok=True)
    p = lambda ext: os.path.join(out_dir, f"{name}.{ext}")
    if redownload or not os.path.exists(p("raw.osm.xml")):
        print("[1/4] OSM 다운로드...")
        download(lat, lon, radius, p("raw.osm.xml"))
    print("[2/4] 반경 밖 잘라내기...")
    clip(p("raw.osm.xml"), p("osm.xml"), lat, lon, radius)

    # 큰 도로만 남겨 변환하고, 결과를 plain XML(노드/엣지 목록)로 받는다.
    # 좌표는 중심 좌표가 (0,0)이 되도록 투영 (기존 십자 교차로와 같은 관례).
    print("[3/4] netconvert (큰 도로만)...")
    plain = os.path.join(out_dir, f"{name}.plain")
    subprocess.run(
        [checkBinary("netconvert"),
         "--osm-files", p("osm.xml"),
         "--type-files", os.path.join(TYPEMAP, "osmNetconvert.typ.xml"),
         "--keep-edges.by-type", ",".join("highway." + t for t in MAJOR_TYPES),
         "--proj", tmerc(lat, lon), "--offset.disable-normalization", "true",
         "--geometry.remove", "--roundabouts.guess", "--ramps.guess",
         "--junctions.join", "--tls.guess-signals", "--tls.join",
         "--remove-edges.isolated", "--keep-edges.components", "1",
         "--output.street-names", "--osm.lane-access", "false",
         "--no-turnarounds.except-deadend", "--no-warnings",
         "-o", p("major.net.xml"), "--plain-output-prefix", plain],
        check=True)

    print("[4/4] road_config.py 에 쓰기...")
    nodes = {n.get("id"): (float(n.get("x")), float(n.get("y")), n.get("type", "priority"))
             for n in ET.parse(plain + ".nod.xml").getroot().findall("node")}
    edges = {e.get("id"): e for e in ET.parse(plain + ".edg.xml").getroot().findall("edge")}

    # 양방향 도로 묶기: from/to가 뒤바뀌고 중심선이 같은(역순) 엣지 쌍을
    # 하나의 도로(lanes=(정방향, 역방향))로 합친다.
    # (id로 짝을 찾으면 안 된다: clip()이 만든 음수 way id는 변환 후
    #  "-37#0" / "--37#1"처럼 끝 번호까지 달라져 짝을 놓친다 → 일방통행 두 개가
    #  같은 중심선 위에 겹쳐 그려짐)
    def _same_line(a, b):
        sa = _shape(a.get("shape")) if a.get("shape") else []
        sb = _shape(b.get("shape")) if b.get("shape") else []
        return len(sa) == len(sb) and all(
            math.dist(p, q) < 0.5 for p, q in zip(sa, reversed(sb)))

    by_ends = {}
    for e in edges.values():
        by_ends.setdefault((e.get("from"), e.get("to")), []).append(e)
    roads, used = [], set()
    for eid in sorted(edges, key=len):     # 짧은 id(정방향)부터
        e = edges[eid]
        if eid in used:
            continue
        rev = next((c for c in by_ends.get((e.get("to"), e.get("from")), [])
                    if c.get("id") not in used and c.get("id") != eid and _same_line(e, c)),
                   None)
        used.add(eid)
        if rev is not None:
            used.add(rev.get("id"))
        fr, to = e.get("from"), e.get("to")
        shape = _shape(e.get("shape")) if e.get("shape") else [nodes[fr][:2], nodes[to][:2]]
        via = _simplify(shape, 0.5)[1:-1]
        sp_f = round(float(e.get("speed")), 2)
        sp_b = round(float(rev.get("speed")), 2) if rev is not None else sp_f
        roads.append({
            "name": e.get("name", ""),
            "type": e.get("type", "").replace("highway.", ""),
            "from": fr, "to": to,
            "lanes": (int(e.get("numLanes", 1)), int(rev.get("numLanes", 1)) if rev is not None else 0),
            "speed": sp_f if sp_f == sp_b else (sp_f, sp_b),
            "via": [(round(x, 1), round(y, 1)) for x, y in via],
        })

    # 사람이 읽기 쉬운 id로 바꾼다: 노드는 중심에서 가까운 순 J1, J2...
    # 도로는 등급 → 이름 → 시작 노드 순 R1, R2...
    order = sorted(nodes, key=lambda n: math.hypot(nodes[n][0], nodes[n][1]))
    nid = {old: f"J{i + 1}" for i, old in enumerate(order)}
    rank = {t: i for i, t in enumerate(MAJOR_TYPES)}
    roads.sort(key=lambda r: (rank.get(r["type"], 99), r["name"], int(nid[r["from"]][1:])))
    touching = {n: [] for n in nodes}
    for r in roads:
        for n in (r["from"], r["to"]):
            if r["name"] and r["name"] not in touching[n]:
                touching[n].append(r["name"])

    L = [BLOCK_BEGIN,
         "# ──────────────────────────────────────────────────────────────",
         "# 4) 실제 지도(OSM) 도로 — ROAD[\"layout\"] = \"osm\" 일 때 사용",
         "#",
         f"#    env/osm_builder.py --to-config 가 자동 생성 ({name}, 반경 {radius:.0f} m).",
         "#    다시 생성하면 이 블록(>>> ~ <<<)이 통째로 덮어써지니,",
         "#    손으로 고친 값을 지키려면 재생성 전에 따로 백업할 것.",
         "#",
         "#    좌표: 중심 좌표(source.lat/lon)가 (0,0), x=동쪽, y=북쪽 (m)",
         "#    nodes: id → (x, y, 교차로 종류)",
         "#        종류: traffic_light(신호등) / priority(우선도로) / dead_end(막다른 끝)",
         "#              / right_before_left / allway_stop 등 SUMO 노드 타입",
         "#    roads: id → 도로 하나 (양방향이면 SUMO 엣지 2개로 만들어짐)",
         "#        from/to : 양 끝 노드. 정방향 엣지 id = 도로 id, 역방향 = \"-\" + 도로 id",
         "#        lanes   : (정방향 차선 수, 역방향 차선 수). 역방향 0 = 일방통행",
         "#        speed   : 제한속도 (m/s). 방향별로 다르면 (정방향, 역방향)",
         "#        type    : OSM 도로 등급 → 교차로 우선순위 결정 (motorway > trunk > primary ...)",
         "#        via     : 시작/끝 노드 사이의 중간 꺾임점 [(x, y), ...]. 비우면 직선",
         "#    traffic: 배경차 생성 (randomTrips — 도로망 전체에 무작위 출발/도착)",
         "#        배경차 차종은 위 TRAFFIC[\"controllers\"] 믹스를 그대로 쓴다.",
         "# ──────────────────────────────────────────────────────────────",
         "OSM_ROAD = {",
         f'    "source": {{"name": "{name}", "lat": {lat}, "lon": {lon}, "radius": {radius:g},',
         f'               "osm_file": "env/sumo_osm/{name}.osm.xml"}},  # 건물/공원 배경용 (없으면 생략)',
         f'    "proj": "{tmerc(lat, lon)}",',
         '    "traffic": {',
         '        "vehs_per_hour": 900,   # 도로망 전체에 시간당 투입 대수',
         '        "min_distance": 300,    # 출발~도착 최소 직선거리 (m)',
         '        "fringe_factor": 5,     # 지도 가장자리에서 출발/도착할 확률 배율',
         '        "end": 3600,            # 생성할 시간 범위 (s)',
         '        "seed": 42,',
         '    },',
         '    # 보행자: 인도 + 교차로 횡단보도 + 걷는 사람',
         '    #   인도가 생기면 각 엣지의 맨 오른쪽(차선 index 0)이 인도가 되고',
         '    #   차량 차선 index는 1부터 시작한다.',
         '    #   도로 하나만 인도를 빼거나 폭을 바꾸려면 그 도로에 "sidewalk": 0 (또는 폭 m)을 준다.',
         '    "pedestrians": {',
         '        "enabled": True,',
         '        "sidewalk_width": 3.0,  # 인도 폭 (m)',
         '        "sidewalk_types": ["primary", "primary_link", "secondary", "secondary_link",',
         '                           "tertiary", "tertiary_link"],  # 인도를 붙일 도로 등급',
         '                                                          # (강변북로 등 trunk/램프는 제외)',
         '        "crossing_nodes": "traffic_light",  # 횡단보도를 만들 교차로.',
         '                                            # "traffic_light" = 신호등 교차로 전부,',
         '                                            # 또는 ["J1", "J42"] 처럼 노드 목록',
         '        "persons_per_hour": 600,  # 시간당 투입 보행자 수 (도로망 전체)',
         '        "min_distance": 150,      # 출발~도착 최소 직선거리 (m)',
         '        "seed": 7,',
         '        # 교통섬(도로·인도로 둘러싸인 빈 땅)을 인도처럼 포장해서 그린다.',
         '        #   섬 안쪽의 아무 점 (x, y)만 적으면 둘러싼 영역을 자동으로 찾아 채운다.',
         '        #   ※ 보기용: 보행자는 섬 둘레 인도로 다니고, 섬 안쪽을 가로지르지는 않는다.',
         '        "islands": [],',
         '        # 교차로가 아닌 도로 중간 횡단보도 (주로 우회전 연결로 → 교통섬).',
         '        #   점 (x, y)에서 가장 가까운 도로를 끊어 횡단보도를 놓고, 교통섬을 가로지르는',
         '        #   보행로로 섬 꼭짓점 교차로의 인도에 잇는다. 이어질 노드를 직접 정하려면',
         '        #   {"at": (x, y), "to": "J1"} 처럼 준다. (끊긴 도로 뒷도막 id = 원래 id + ".c")',
         '        "midblock_crossings": [],',
         '    },',
         '    "nodes": {']
    for old in order:
        x, y, t = nodes[old]
        tail = f'  # {" · ".join(touching[old][:3])}' if touching[old] else ""
        L.append(f'        "{nid[old]}": ({x:.1f}, {y:.1f}, "{t}"),{tail}')
    L.append("    },")
    L.append('    "roads": {')
    for i, r in enumerate(roads):
        L.append(f'        "R{i + 1}": {{"name": "{r["name"]}", "type": "{r["type"]}", '
                 f'"from": "{nid[r["from"]]}", "to": "{nid[r["to"]]}", '
                 f'"lanes": {r["lanes"]}, "speed": {r["speed"]},')
        L.append(f'               "via": [{", ".join(f"({x}, {y})" for x, y in r["via"])}]}},')
    L += ["    },",
          "    # 이름 붙인 경로: 이름 → edges(순서대로 지나갈 엣지 id). 역방향 엣지는 \"-\" + 도로 id.",
          "    #   SUMO에 route \"r_이름\" 으로 들어간다. 경로만 정의하며, 이 경로를 달리는 차는",
          "    #   따로 지정해야 한다. 끊긴 경로나 없는 엣지는 빌드할 때 오류가 난다.",
          '    # 차선 연결 직접 지정: (from 엣지, from 차선, to 엣지, to 차선). 차선 0번은 인도.',
          '    "connections": [],',
          '    # 신호 시간 조정: 노드 id → {"green": [{"from", "to", "seconds"}], "cycle": 초}',
          '    "signals": {},',
          '    # 화면에서 정지선처럼 보이는 표시를 가리고 차선을 이어 그릴 교차로 (보기용).',
          '    "seamless": [],',
          '    "routes": {},',
          "    # 버스 정류장: 이름 → road(도로 id), at(정류장 위치 근처 좌표), length(정차 구간 길이 m)",
          "    #   at에서 가장 가까운 그 도로의 지점을 중심으로 length만큼 정차 구간을 만든다.",
          '    "bus_stops": {},',
          "}", BLOCK_END]
    block = "\n".join(L) + "\n"

    with open(config_path, encoding="utf-8") as f:
        src = f.read()
    if BLOCK_BEGIN in src and BLOCK_END in src:
        head, rest = src.split(BLOCK_BEGIN, 1)
        src = head + block + rest.split(BLOCK_END, 1)[1].lstrip("\n")
    else:
        src = src.rstrip("\n") + "\n\n\n" + block
    with open(config_path, "w", encoding="utf-8") as f:
        f.write(src)
    print(f"완료: 노드 {len(nodes)}개, 도로 {len(roads)}개 → {config_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--radius", type=float, default=1000.0, help="반경 (m)")
    ap.add_argument("--name", default="osm")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "sumo_osm"))
    ap.add_argument("--period", type=float, default=2.0, help="배경 차량 생성 간격 (초)")
    ap.add_argument("--end", type=int, default=3600)
    ap.add_argument("--redownload", action="store_true")
    ap.add_argument("--to-config", action="store_true",
                    help="큰 도로만 골라 env/road_config.py 의 OSM_ROAD 블록으로 쓴다")
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "road_config.py"))
    a = ap.parse_args()
    if a.to_config:
        export_config(a.lat, a.lon, a.radius, a.name, a.out, a.config, a.redownload)
    else:
        build(a.lat, a.lon, a.radius, a.name, a.out, a.period, a.end, a.redownload)
