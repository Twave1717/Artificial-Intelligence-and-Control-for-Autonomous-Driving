"""configs/road_config.py 를 읽어 SUMO 도로 파일들을 자동 생성하는 모듈.

SUMO는 도로를 여러 XML 파일로 정의한다:
    highway.nod.xml  : 노드(교차점/도로 끝점)의 좌표
    highway.edg.xml  : 엣지(도로 구간) — 어느 노드에서 어느 노드로, 차선 수, 제한속도
    highway.net.xml  : 위 두 파일을 netconvert 도구로 컴파일한 "완성된 도로망"
                       (SUMO가 실제로 읽는 파일. 직접 작성하기엔 복잡해서 도구로 생성)
    highway.rou.xml  : 차량 타입(vType)과 교통 수요(flow/route) 정의
    highway.sumocfg  : 위 파일들을 묶어주는 설정 파일 (시뮬레이션 실행의 진입점)

train.py / test.py 가 시작할 때마다 build()를 호출하므로,
road_config.py만 수정하면 도로가 자동으로 다시 만들어진다.
"""
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from sumolib import checkBinary  # SUMO 실행파일(sumo, netconvert 등) 경로를 찾아주는 유틸
from env.mdp_config import SIMULATION
try:
    from env.road_config import SCENERY
except ImportError:      # 구버전 설정 하위호환
    SCENERY = {"trees": False}


def traffic_routes(traffic: dict, ego: dict) -> dict:
    """TRAFFIC["routes"]를 기본값이 채워진 형태로 돌려준다.

    반환: {이름: {"edges": [...], "vehs_per_hour": float, "depart_lane": str}}
    routes 설정이 없으면(구버전 설정) ego 경로와 같은 "main" 하나만 만든다.
    road_builder(flow 생성)와 sumo_env(프리필)가 같은 해석을 쓰도록 여기 하나로 둔다.
    """
    routes = traffic.get("routes") or {
        "main": {"edges": list(ego.get("route", ["w2c", "c2e"]))}}
    out = {}
    for name, r in routes.items():
        vph = r.get("vehs_per_hour")
        out[name] = {
            "edges": list(r["edges"]),
            "vehs_per_hour": float(traffic["vehs_per_hour"] if vph is None else vph),
            "depart_lane": str(r.get("depart_lane", traffic["depart_lane"])),
        }
    return out


def build(road: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """SUMO 파일들을 out_dir에 생성하고, sumocfg 파일 경로를 반환한다.

    Args:
        road:    configs.road_config.ROAD    (도로 형상)
        ego:     configs.road_config.EGO     (RL 차량 속성)
        traffic: configs.road_config.TRAFFIC (배경 교통류)
        out_dir: 생성 위치 (보통 프로젝트의 sumo/ 폴더)
    """
    os.makedirs(out_dir, exist_ok=True)

    # 실제 지도 모드: road_config.OSM_ROAD 의 노드/도로 목록으로 만든다
    if road.get("layout", "cross") == "osm":
        from env.road_config import OSM_ROAD
        return _build_osm(OSM_ROAD, ego, traffic, out_dir)

    # ──────────────────────────────────────────────────────
    # 1) 노드 파일: 신호등 교차로(중앙) + 동서남북 도로 끝점 4개
    #    교차로 중심이 (0,0). 주 도로는 x축, 교차 도로는 y축 방향.
    #    type="traffic_light" → netconvert가 고정주기 신호 프로그램을 자동 생성.
    # ──────────────────────────────────────────────────────
    half = float(road["length"]) / 2.0
    cross_half = float(road.get("cross_length", road["length"])) / 2.0
    cross_lanes = int(road.get("cross_num_lanes", road["num_lanes"]))
    cross_speed = float(road.get("cross_speed_limit", road["speed_limit"]))
    with open(os.path.join(out_dir, "highway.nod.xml"), "w") as f:
        f.write(f"""<nodes>
    <node id="n_center" x="0" y="0" type="traffic_light"/>
    <node id="n_west"   x="{-half}" y="0"/>
    <node id="n_east"   x="{half}"  y="0"/>
    <node id="n_north"  x="0" y="{cross_half}"/>
    <node id="n_south"  x="0" y="{-cross_half}"/>
</nodes>
""")

    # ──────────────────────────────────────────────────────
    # 2) 엣지 파일: 방향별 단방향 엣지 8개 (양방향 도로 = 엣지 2개)
    #    numLanes = 한 방향 차선 수, speed = 제한속도(m/s)
    #    같은 두 노드 사이의 반대 방향 엣지는 netconvert가 각자 진행 방향
    #    오른쪽에 붙여 그린다 → 우측통행.
    # ──────────────────────────────────────────────────────
    main = f'numLanes="{road["num_lanes"]}" speed="{road["speed_limit"]}"'
    cross = f'numLanes="{cross_lanes}" speed="{cross_speed}"'
    with open(os.path.join(out_dir, "highway.edg.xml"), "w") as f:
        f.write(f"""<edges>
    <edge id="w2c" from="n_west"   to="n_center" {main}/>
    <edge id="c2e" from="n_center" to="n_east"   {main}/>
    <edge id="e2c" from="n_east"   to="n_center" {main}/>
    <edge id="c2w" from="n_center" to="n_west"   {main}/>
    <edge id="n2c" from="n_north"  to="n_center" {cross}/>
    <edge id="c2s" from="n_center" to="n_south"  {cross}/>
    <edge id="s2c" from="n_south"  to="n_center" {cross}/>
    <edge id="c2n" from="n_center" to="n_north"  {cross}/>
</edges>
""")

    # ──────────────────────────────────────────────────────
    # 3) 라우트 파일: 차량 타입 2종 + 배경 교통 flow
    #
    #    <vType>  : 차량의 물리 속성 템플릿
    #       ego_type : RL이 조종할 차 (빨간색, color="1,0,0" = RGB)
    #    <vTypeDistribution> : 배경차 컨트롤러 믹스.
    #       TRAFFIC["controllers"]에 정의된 여러 차량추종모델(Krauss/IDM/
    #       EIDM/ACC 등)을 probability 비율로 섞는다. flow가 이 분포를
    #       type으로 쓰면, 투입되는 차량마다 SUMO가 확률적으로 타입을 뽑는다.
    #    <route>  : 지나갈 엣지들의 나열. TRAFFIC["routes"]의 경로마다 하나씩.
    #    <flow>   : 교통 수요 정의 (경로마다 하나). period="X"는 "X초마다 정확히 1대씩"
    #               → 등간격(uniform) 생성. (vehsPerHour와 등가지만
    #               "비슷한 간격으로 새로 생성"을 명시적으로 보장)
    #               ego는 여기에 없고, 환경(sumo_env.py)이 reset마다
    #               traci.vehicle.add()로 직접 투입한다.
    # ──────────────────────────────────────────────────────
    vtypes_block, flow_type = _vtypes(traffic)

    route_lines = []
    for name, r in traffic_routes(traffic, ego).items():
        route_lines.append(f'    <route id="r_{name}" edges="{" ".join(r["edges"])}"/>')
        if r["vehs_per_hour"] > 0:
            route_lines.append(
                f'    <flow id="flow_{name}" type="{flow_type}" route="r_{name}" '
                f'begin="0" end="100000"\n'
                f'          period="{3600.0 / r["vehs_per_hour"]:.2f}"\n'
                f'          departLane="{r["depart_lane"]}" '
                f'departSpeed="{traffic["depart_speed"]}"/>')
    routes_block = "\n".join(route_lines)

    with open(os.path.join(out_dir, "highway.rou.xml"), "w") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
{routes_block}
</routes>
""")
    return _finish_cross(road, ego, out_dir)


def _vtypes(traffic: dict, allow_lane_keep: bool = True):
    """배경차 vType(분포) XML 블록과 flow/trip에 쓸 type id를 돌려준다.

    allow_lane_keep=False면 controllers의 keep_lane 설정을 무시한다
    (실제 지도처럼 회전이 많은 도로망에선 차선변경을 끄면 회전 차선으로
     못 들어가 교차로에서 막혀버리기 때문).
    """
    controllers = traffic.get("controllers")
    if controllers:
        # probability 합이 1이 아니어도 되도록 자동 정규화
        total_p = sum(float(c.get("probability", 1.0))
                      for c in controllers.values())
        vtype_lines = []
        for name, c in controllers.items():
            prob = float(c.get("probability", 1.0)) / total_p
            attrs = [
                f'id="car_{name}"',
                f'probability="{prob:.4f}"',
                f'carFollowModel="{c["carFollowModel"]}"',
                f'accel="{c.get("accel", 2.5)}"',
                f'decel="{c.get("decel", 4.5)}"',
                f'tau="{c.get("tau", 1.0)}"',
                f'minGap="{c.get("min_gap", 2.5)}"',
                # 컨트롤러별 희망속도 오버라이드 (없으면 공통 max_speed).
                # 컨트롤러마다 속도를 다르게 주면 차선 간 속도차가 생겨
                # "추월할 이유"가 자연스럽게 만들어진다.
                f'maxSpeed="{c.get("max_speed", traffic["max_speed"])}"',
                # speedDev: 같은 타입 안에서도 개체별 희망속도 편차 (0~)
                f'speedDev="{c.get("speed_dev", 0.1)}"',
                f'color="{c.get("color", "1,1,0")}"',
                'vClass="passenger"', 'guiShape="passenger/sedan"',
            ]
            # sigma는 Krauss 계열에만 의미가 있으므로 지정된 경우에만 넣는다
            if "sigma" in c:
                attrs.append(f'sigma="{c["sigma"]}"')
            # ── 배경차 차선 유지 (keep_lane, 기본 True) ──
            # 배경차의 자율 차선변경(전략/협조/속도이득/우측유지)을 전부 꺼서
            # 각자 자기 차선을 지키게 한다. 이유:
            #   배경차가 자유롭게 돌아다니면 (측정: 150스텝에 477회 변경)
            #   차선 간 틈이 수시로 닫혀서, ego가 틈을 노려 파고드는
            #   "칼치기 추월"을 학습할 안정적인 기회 자체가 사라진다.
            #   배경차가 차선을 유지하면 "느린 차 뒤 → 옆 틈으로 변경 →
            #   가속"이라는 추월 패턴이 재현 가능한 형태로 나타난다.
            # 특정 컨트롤러만 돌아다니게 하려면 그 항목에 "keep_lane": False.
            if allow_lane_keep and c.get("keep_lane", True):
                attrs += ['lcStrategic="0"', 'lcCooperative="0"',
                          'lcSpeedGain="0"', 'lcKeepRight="0"']
            vtype_lines.append("        <vType " + " ".join(attrs) + "/>")
        vtypes_block = ('    <vTypeDistribution id="carDist">\n'
                        + "\n".join(vtype_lines)
                        + "\n    </vTypeDistribution>")
        flow_type = "carDist"
    else:
        # controllers 설정이 없으면 예전 방식(단일 Krauss)으로 동작 (하위호환)
        vtypes_block = (f'    <vType id="car" accel="2.5" decel="4.5" '
                        f'maxSpeed="{traffic["max_speed"]}" '
                        f'sigma="{traffic.get("sigma", 0.5)}" '
                        f'vClass="passenger" guiShape="passenger/sedan"/>')
        flow_type = "car"
    return vtypes_block, flow_type


def _finish_cross(road: dict, ego: dict, out_dir: str) -> str:
    """십자 교차로 모드의 나머지 단계: GUI 설정, 장식, sumocfg, netconvert."""
    half = float(road["length"]) / 2.0

    # ──────────────────────────────────────────────────────
    # 4) GUI 기본 시각화 설정
    #    GUI 관련 값의 source of truth는 env/mdp_config.py의 SIMULATION 하나뿐이다.
    #    view_road.py / train.py / test.py / 데이터 수집 모두 build()를 거치므로
    #    gui_zoom/gui_delay를 바꾸면 생성되는 highway.gui.xml에도 즉시 반영된다.
    # ──────────────────────────────────────────────────────
    # GUI 시작 화면: "도로 시작 구간"이 차선까지 보이게 자동 확대해둔다.
    #   (12km 전체 맞춤으로 시작하면 도로가 가느다란 선으로만 보여 직관성이
    #    떨어지므로, ego가 등장할 시작부를 확대해 대기 화면으로 삼는다.
    #    ▶ 재생 후에는 env가 ego를 추적(trackVehicle)하며 시점을 가져간다.)
    # 좌표/배율은 도로 길이·차선 수에서 "계산"하므로 설정을 바꿔도 안 어긋난다:
    #   zoom(%) = 도로길이 / 보고싶은폭(gui_view_width) × 100
    #     (SUMO zoom은 "네트워크 전체 = 100%" 기준)
    #   x 시작 = 서쪽 끝(-length/2) + ego 출발 위치
    #   y 중심 = -차선폭(3.2m) × 차선수 / 2   (동쪽행 차선은 y<0 쪽에 그려짐)
    gui_delay = float(SIMULATION.get("gui_delay", 100.0))
    view_width = float(SIMULATION.get("gui_view_width", 500.0))   # 시작 화면 시야(m)
    zoom = float(road["length"]) / view_width * 100.0
    center_x = (-half + float(ego.get("depart_pos", 0))
                + view_width * 0.4)                               # 시작점이 화면 왼쪽에 오게
    center_y = -3.2 * int(road["num_lanes"]) / 2.0
    gui_settings_path = os.path.join(out_dir, "highway.gui.xml")
    with open(gui_settings_path, "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world"/>
    <viewport zoom="{zoom:.0f}" x="{center_x:.1f}" y="{center_y:.1f}" angle="0"/>
    <delay value="{gui_delay}"/>
</viewsettings>
""")

    # ──────────────────────────────────────────────────────
    # 4.7) 도로변 장식(나무) — 순수 시각 요소
    #
    # SUMO의 <poly>(다각형)로 "초록 수관 + 갈색 줄기"를 그려 도로
    # 양옆에 심는다. additional 파일로 로드되며 차량 물리와 완전히
    # 무관하다 (충돌/관측/보상 어디에도 안 잡힘).
    # 좌표계: 주 도로는 동쪽행 y ∈ [-3.2×차선수, 0], 서쪽행 y ∈ [0, 3.2×차선수]
    # → 숲은 그 바깥(± side_offset)에 배치하고, 교차 도로 구간은 비운다.
    # ──────────────────────────────────────────────────────
    _write_scenery(out_dir, road)

    # ──────────────────────────────────────────────────────
    # 5) sumocfg: "이 net 파일과 이 route 파일로 시뮬레이션해라"
    # ──────────────────────────────────────────────────────
    cfg_path = os.path.join(out_dir, "highway.sumocfg")
    with open(cfg_path, "w") as f:
        f.write("""<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="highway.rou.xml"/>
        <additional-files value="highway.scenery.xml"/>
    </input>
    <gui_only>
        <gui-settings-file value="highway.gui.xml"/>
    </gui_only>
</configuration>
""")

    # ──────────────────────────────────────────────────────
    # 6) netconvert 실행: nod + edg → net.xml 컴파일
    #    (SUMO 설치 시 함께 깔리는 커맨드라인 도구)
    #    --offset.disable-normalization: 좌표를 노드 파일 그대로 유지
    #    (기본값은 최소 좌표가 0이 되게 전체를 평행이동시켜서, 교차로가
    #     (0,0)이 아니게 되고 GUI 시점/나무 좌표가 어긋난다)
    # ──────────────────────────────────────────────────────
    netconvert = checkBinary("netconvert")
    subprocess.run(
        [netconvert,
         "--node-files", os.path.join(out_dir, "highway.nod.xml"),
         "--edge-files", os.path.join(out_dir, "highway.edg.xml"),
         "--offset.disable-normalization", "true",
         "-o", os.path.join(out_dir, "highway.net.xml")],
        check=True,           # 실패하면 예외를 던져서 바로 알 수 있게
        capture_output=True,  # 출력은 조용히 삼킴
    )
    return cfg_path


# OSM 도로 등급 → 교차로 우선순위 (SUMO osmNetconvert.typ.xml 과 같은 값).
# 숫자가 큰 도로가 교차로에서 우선권을 갖는다.
OSM_PRIORITY = {
    "motorway": 14, "trunk": 13, "primary": 12, "secondary": 11, "tertiary": 10,
    "motorway_link": 9, "trunk_link": 8, "primary_link": 7,
    "secondary_link": 6, "tertiary_link": 5,
    "busway": 12,       # 중앙버스전용차로 (직접 추가하는 도로용)
    "residential": 3, "service": 2,     # 골목 (직접 추가하는 도로용)
}


def _road_pts(r: dict, nodes: dict) -> list:
    """도로 중심선 점 목록. 양 끝은 노드 좌표이고, 도로에 "start"/"end"가 있으면
    그 좌표를 쓴다 (차선 수가 달라지는 곳에서 한쪽 가장자리를 맞출 때).
    "smooth": True면 꺾인 점을 곡선으로 다듬는다 (양 끝점은 그대로)."""
    pts = ([tuple(r.get("start", nodes[r["from"]][:2]))]
           + [tuple(v) for v in r.get("via", [])]
           + [tuple(r.get("end", nodes[r["to"]][:2]))])
    if r.get("smooth"):
        # Chaikin 모서리 깎기: 꺾인 점마다 양쪽 25% 지점으로 바꾸기를 3번 반복
        for _ in range(3):
            out = [pts[0]]
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                out.append((0.75 * x1 + 0.25 * x2, 0.75 * y1 + 0.25 * y2))
                out.append((0.25 * x1 + 0.75 * x2, 0.25 * y1 + 0.75 * y2))
            pts = out + [pts[-1]]
        pts = [(round(x, 2), round(y, 2)) for x, y in pts]
    return pts


def _plan_midblock(nodes: dict, roads: dict, points, walkways: bool = True) -> dict:
    """도로 중간 횡단보도 계획: 점마다 가장 가까운 도로와 그 위의 분할 지점,
    그리고 교통섬 쪽 보행로가 이어질 노드를 정한다.

    보행로 도착 노드: 분할되는 도로의 양 끝 노드와 '둘 다' 도로로 이어진 노드
    (= 그 도로와 함께 삼각형 교통섬을 이루는 꼭짓점) 중 가장 가까운 것.
    점을 {"at": (x, y), "to": "J1"} 로 주면 도착 노드를 직접 지정한다.
    반환: {도로 id: {"node", "walk", "xy", "seg", "target"}}
    """
    nbr = {n: set() for n in nodes}
    for r in roads.values():
        nbr[r["from"]].add(r["to"])
        nbr[r["to"]].add(r["from"])
    plan = {}
    for k, item in enumerate(points, 1):
        pt = item["at"] if isinstance(item, dict) else item
        best = None
        for rid, r in roads.items():
            pts = _road_pts(r, nodes)
            for i, ((x1, y1), (x2, y2)) in enumerate(zip(pts, pts[1:])):
                dx, dy = x2 - x1, y2 - y1
                t = max(0.0, min(1.0, ((pt[0] - x1) * dx + (pt[1] - y1) * dy)
                                 / ((dx * dx + dy * dy) or 1e-9)))
                q = (x1 + t * dx, y1 + t * dy)
                d = math.dist(pt, q)
                if best is None or d < best[0]:
                    best = (d, rid, i, q)
        _, rid, i, q = best
        if rid in plan:
            print(f"  [횡단보도] {pt}: {rid}에는 이미 도로 중간 횡단보도가 있어 건너뜀")
            continue
        r = roads[rid]
        target = item.get("to") if isinstance(item, dict) else None
        if not walkways:
            # 섬 보행로 없이 횡단보도만 (섬 쪽 보행은 연결로 안쪽 보행 차선이 맡음)
            plan[rid] = {"node": f"C{k}", "walk": None, "xy": q, "seg": i, "target": None, "via": []}
            continue
        if target is None:
            common = (nbr[r["from"]] & nbr[r["to"]]) - {r["from"], r["to"]}
            if not common:
                print(f"  [횡단보도] {pt}: {rid} 옆에 교통섬 꼭짓점이 없어 건너뜀")
                continue
            target = min(common, key=lambda n: math.dist(q, nodes[n][:2]))
        plan[rid] = {"node": f"C{k}", "walk": f"W{k}", "xy": q, "seg": i, "target": target,
                     "via": [tuple(v) for v in item.get("via", [])] if isinstance(item, dict) else []}
    return plan


def _seam_polys(net_file: str, junctions, layer: float = 5.0) -> list:
    """교차로 안에서 끊긴 직진 차선 점선을 이어 그리는 보기용 선.

    교차로 규칙은 그대로 두고(차량 동작 불변) 화면에서만 도로가 이어져 보이게 한다.
    (우선 도로 끝의 흰 표시 막대는 GUI 설정 realisticLinkRules로 숨긴다)
    """
    import sumolib

    net = sumolib.net.readNet(net_file, withInternal=True)
    polys = []
    centers = set()     # 중앙선을 이미 그린 도로 쌍 (양방향이라 두 번 그리지 않게)

    def line(pts, color, width):
        shape = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
        polys.append(f'<poly id="seam_{len(polys)}" type="seam" color="{color}" fill="0" '
                     f'lineWidth="{width}" layer="{layer}" shape="{shape}"/>')

    def resample(pts, step=0.5):
        out = []
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.dist(a, b) / step))
            out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
        return out + [pts[-1]]

    for nid in junctions:
        for edge in net.getNode(nid).getIncoming():
            if edge.getFunction():
                continue
            through = {}        # (도착 엣지, 도착 차선) → 교차로 내부 차선 모양
            for lane in edge.getLanes():
                if not lane.allows("passenger"):
                    continue
                for c in lane.getOutgoing():
                    via = c.getViaLaneID()
                    if via and c.getDirection() == "s":
                        through[lane.getIndex()] = (c.getToLane(), net.getLane(via).getShape())
            # 중앙선: 양방향 도로의 맨 왼쪽 차선 왼쪽 경계를 교차로 안으로 잇는다
            if through:
                to_top, via_top = through[max(through)]
                rev = lambda eid: eid[1:] if eid.startswith("-") else "-" + eid
                key = frozenset((edge.getID(), rev(edge.getID()),
                                 to_top.getEdge().getID(), rev(to_top.getEdge().getID())))
                if (net.hasEdge(rev(edge.getID())) and net.hasEdge(rev(to_top.getEdge().getID()))
                        and key not in centers):
                    centers.add(key)
                    half = to_top.getWidth() / 2.0
                    offset = []
                    for (x1, y1), (x2, y2) in zip(via_top, via_top[1:]):
                        d = math.dist((x1, y1), (x2, y2))
                        if d < 0.5:         # 끝의 아주 짧은 조각은 방향이 튀어 갈고리처럼 보인다
                            continue
                        nx, ny = -(y2 - y1) / d * half, (x2 - x1) / d * half
                        offset += [(x1 + nx, y1 + ny), (x2 + nx, y2 + ny)]
                    if len(offset) >= 2:
                        line(offset, "255,255,255", 0.1)
            for i in sorted(through):
                if i + 1 not in through:
                    continue
                (to_a, a), (to_b, b) = through[i], through[i + 1]
                if to_a.getEdge() != to_b.getEdge():
                    continue
                pa, pb = resample(a), resample(b)
                k = min(len(pa), len(pb))
                mid = [((pa[int(j * (len(pa) - 1) / (k - 1))][0] + pb[int(j * (len(pb) - 1) / (k - 1))][0]) / 2,
                        (pa[int(j * (len(pa) - 1) / (k - 1))][1] + pb[int(j * (len(pb) - 1) / (k - 1))][1]) / 2)
                       for j in range(k)]
                for j in range(1, k - 6, 12):      # 0.5m 간격: 3m 그리고 3m 띄움
                    line(mid[j:j + 7], "255,255,255", 0.1)
    return polys


def _island_polygons(net_file: str, seeds, cell: float = 0.25, box: float = 60.0):
    """seed 점을 둘러싼 '도로/인도로 막힌 빈 땅'(교통섬)의 외곽 다각형을 구한다.

    방법(래스터 flood fill):
      1) seed 주변 ±box m를 cell 간격 격자로 나누고, 차선(폭 포함)·교차로·보행공간이
         덮는 칸을 '막힘'으로 표시
      2) seed에서 막히지 않은 칸으로 번져나가며 영역을 채움
         (격자 끝까지 번지면 닫힌 섬이 아니므로 건너뜀)
      3) 영역을 살짝 키운 뒤 볼록 껍질(convex hull)을 다각형으로 돌려준다.
         도로보다 아래층에 그리므로 가장자리가 도로와 겹쳐도 가려진다.
    """
    import numpy as np
    import sumolib

    net = sumolib.net.readNet(net_file, withInternal=True)
    shapes = []      # (폴리라인 점들, 반폭) 또는 (다각형 점들, None)
    for edge in net.getEdges(withInternal=True):
        # 섬을 가로지르는 보행로(보행자 전용 엣지)는 섬을 둘로 쪼개지 않게 제외
        if edge.getFunction() == "" and all(
                l.allows("pedestrian") and not l.allows("passenger") for l in edge.getLanes()):
            continue
        for lane in edge.getLanes():
            if edge.getFunction() == "walkingarea":
                shapes.append((lane.getShape(), None))
            else:
                shapes.append((lane.getShape(), lane.getWidth() / 2.0))
    for node in net.getNodes():
        if node.getShape():
            shapes.append((node.getShape(), None))

    def in_poly(px, py, poly):
        inside = np.zeros(px.shape, dtype=bool)
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            if y1 == y2:
                continue
            cross = ((py >= min(y1, y2)) & (py < max(y1, y2))
                     & (px < x1 + (py - y1) * (x2 - x1) / (y2 - y1)))
            inside ^= cross
        return inside

    out = []
    for sx, sy in seeds:
        n = int(2 * box / cell)
        xs = sx - box + (np.arange(n) + 0.5) * cell
        ys = sy - box + (np.arange(n) + 0.5) * cell
        px, py = np.meshgrid(xs, ys)
        blocked = np.zeros(px.shape, dtype=bool)
        for pts, hw in shapes:
            xs_, ys_ = zip(*pts)
            if (max(xs_) < sx - box - 10 or min(xs_) > sx + box + 10
                    or max(ys_) < sy - box - 10 or min(ys_) > sy + box + 10):
                continue
            if hw is None:
                if len(pts) >= 3:
                    blocked |= in_poly(px, py, pts)
                continue
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                dx, dy = x2 - x1, y2 - y1
                L2 = dx * dx + dy * dy or 1e-9
                t = np.clip(((px - x1) * dx + (py - y1) * dy) / L2, 0, 1)
                blocked |= (px - x1 - t * dx) ** 2 + (py - y1 - t * dy) ** 2 <= hw * hw
        i0, j0 = n // 2, n // 2
        if blocked[i0, j0]:
            print(f"  [교통섬] ({sx}, {sy}) 은 도로/인도 위라서 건너뜀")
            continue
        region = np.zeros_like(blocked)
        region[i0, j0] = True
        free = ~blocked
        while True:
            grown = region.copy()
            grown[1:] |= region[:-1]; grown[:-1] |= region[1:]
            grown[:, 1:] |= region[:, :-1]; grown[:, :-1] |= region[:, 1:]
            grown &= free
            if (grown == region).all():
                break
            region = grown
        if region[0].any() or region[-1].any() or region[:, 0].any() or region[:, -1].any():
            print(f"  [교통섬] ({sx}, {sy}) 주변이 닫혀 있지 않아 건너뜀")
            continue
        for _ in range(3):      # 0.75m 키워서 도로 가장자리 밑으로 밀어 넣는다
            g = region.copy()
            g[1:] |= region[:-1]; g[:-1] |= region[1:]
            g[:, 1:] |= region[:, :-1]; g[:, :-1] |= region[:, 1:]
            region = g
        pts = sorted(set(zip(px[region].round(2), py[region].round(2))))

        def half(points):
            h = []
            for q in points:
                while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (q[1] - h[-2][1])
                                       - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0])) <= 0:
                    h.pop()
                h.append(q)
            return h
        hull = half(pts)[:-1] + half(pts[::-1])[:-1]
        out.append([(float(x), float(y)) for x, y in hull])
    return out


def _build_osm(osm: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """road_config.OSM_ROAD(노드/도로 목록)로 SUMO 파일들을 만든다.

    파일 이름은 십자 교차로 모드와 같다(highway.*). 달라지는 점:
      - 배경차는 flow 대신 randomTrips로 도로망 전체에 무작위 출발/도착 trip 생성
        (highway.trips.rou.xml). 차종은 TRAFFIC["controllers"] 믹스 그대로.
      - 장식은 나무 대신 OSM 건물/공원 폴리곤 (osm_file이 있을 때만)
    """
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src, nodes, roads = osm["source"], osm["nodes"], osm["roads"]
    p = lambda name: os.path.join(out_dir, name)

    ped = osm.get("pedestrians") or {}
    ped_on = bool(ped.get("enabled", False))
    # island_walkways: False면 횡단보도에서 섬을 가로질러 교차로로 가는 보행로를 만들지 않는다
    #   (보행로가 옆 도로와 나란히 교차로에 들어가면 netconvert가 교차로를 크게 잡기 때문)
    splits = (_plan_midblock(nodes, roads, ped.get("midblock_crossings") or [],
                             ped.get("island_walkways", True)) if ped_on else {})

    # 1) 노드 — <location>을 넣어두면 net이 위경도 기준을 갖게 되어
    #    polyconvert(건물 배경)와 GUI의 위경도 표시가 맞아떨어진다.
    xs = [v[0] for v in nodes.values()]
    ys = [v[1] for v in nodes.values()]
    dlat = float(src["radius"]) / 111320.0
    dlon = dlat / math.cos(math.radians(float(src["lat"])))
    lines = ["<nodes>",
             f'    <location netOffset="0.00,0.00" '
             f'convBoundary="{min(xs):.2f},{min(ys):.2f},{max(xs):.2f},{max(ys):.2f}" '
             f'origBoundary="{src["lon"] - dlon:.6f},{src["lat"] - dlat:.6f},'
             f'{src["lon"] + dlon:.6f},{src["lat"] + dlat:.6f}" '
             f'projParameter="{osm["proj"]}"/>']
    for nid, v in nodes.items():
        # 노드: (x, y, 종류) 또는 (x, y, 종류, {"radius": 교차로 모서리 반경 m})
        x, y, ntype = v[:3]
        radius = f' radius="{v[3]["radius"]}"' if len(v) > 3 and "radius" in v[3] else ""
        # "shape": 교차로 영역을 직접 지정 (정지선 위치를 당기거나 밀 때)
        if len(v) > 3 and v[3].get("shape"):
            radius += ' shape="' + " ".join(f"{px},{py}" for px, py in v[3]["shape"]) + '"'
        # "tl": 신호 묶음 id. 같은 id를 준 신호 교차로들은 신호 프로그램 하나로 같이 움직인다
        if len(v) > 3 and v[3].get("tl"):
            radius += f' tl="{v[3]["tl"]}"'
        lines.append(f'    <node id="{nid}" x="{x}" y="{y}" type="{ntype}"{radius}/>')
    for s in splits.values():      # 도로 중간 횡단보도 지점
        lines.append(f'    <node id="{s["node"]}" x="{s["xy"][0]:.2f}" y="{s["xy"][1]:.2f}" '
                     f'type="priority"/>')
    lines.append("</nodes>")
    with open(p("highway.nod.xml"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # 2) 도로 → 방향별 엣지. 정방향 id = 도로 id, 역방향 = "-" + 도로 id.
    #    양방향 도로는 중심선 기준 각자 오른쪽에 차선을 붙이고(spreadType 기본),
    #    일방통행은 중심선 위에 차선을 펼친다(spreadType="center").
    #    보행자 모드면 인도 폭(sidewalkWidth)을 주고, 인도 없는 도로는
    #    보행자 통행을 막는다(안 막으면 사람이 차도로 걸어 다님).
    #    도로 중간 횡단보도가 있는 도로는 그 지점에서 두 도막으로 나눈다:
    #      정방향 R67 (from→C)  +  R67.c (C→to),  역방향 -R67 / -R67.c
    lines = ["<edges>"]
    for rid, r in roads.items():
        sidewalk = ""
        if ped_on:
            default_w = (float(ped.get("sidewalk_width", 3.0))
                         if r.get("type") in ped.get("sidewalk_types", []) else 0.0)
            w = float(r.get("sidewalk", default_w))
            sidewalk = f' sidewalkWidth="{w}"' if w > 0 else ' disallow="pedestrian"'
        # allow(통행 차종 제한, 예: 버스 전용)이 있으면 그 외는 자동으로 막히므로
        # disallow와 같이 쓰지 않는다 (둘 다 주면 netconvert 오류)
        if r.get("allow"):
            sidewalk = sidewalk if "sidewalkWidth" in sidewalk else ""
            sidewalk += f' allow="{r["allow"]}"'
        fwd, bwd = r["lanes"]
        sp = r["speed"] if isinstance(r["speed"], (tuple, list)) else (r["speed"], r["speed"])
        pts = _road_pts(r, nodes)
        prio = OSM_PRIORITY.get(r.get("type", ""), 1)
        name = f' name="{r["name"]}"' if r.get("name") else ""   # 빈 name은 netconvert 오류
        pieces = [(rid, r["from"], r["to"], fwd, sp[0], pts),
                  ("-" + rid, r["to"], r["from"], bwd, sp[1], pts[::-1])]
        if rid in splits:
            s = splits[rid]
            c, q, i = s["node"], s["xy"], s["seg"]
            a, b = pts[:i + 1] + [q], [q] + pts[i + 1:]
            pieces = [(rid, r["from"], c, fwd, sp[0], a),
                      (rid + ".c", c, r["to"], fwd, sp[0], b),
                      ("-" + rid, r["to"], c, bwd, sp[1], b[::-1]),
                      ("-" + rid + ".c", c, r["from"], bwd, sp[1], a[::-1])]
        for eid, fr, to, n, s, shape in pieces:
            if n <= 0:
                continue
            spread = ' spreadType="center"' if bwd <= 0 else ""
            shape_s = " ".join(f"{x},{y}" for x, y in shape)
            # "end_offset": 정지선을 도로 끝에서 이만큼(m) 뒤로 (정방향만)
            end_off = (f' endOffset="{r["end_offset"]}"'
                       if r.get("end_offset") and not eid.startswith("-") else "")
            head = (f'    <edge id="{eid}" from="{fr}" to="{to}"{name} '
                    f'priority="{prio}" numLanes="{n}" speed="{s}"{spread}{sidewalk}{end_off} '
                    f'shape="{shape_s}"')
            # 차선별 통행 차종 (정방향만): "lane_allow": {차선 번호(인도 포함): "bus"}
            lane_allow = r.get("lane_allow") if not eid.startswith("-") else None
            if lane_allow or "trip_weight" in r:
                lines.append(head + ">")
                # config의 번호는 완성된 도로망 기준(인도 = 0번). netconvert 입력에서는
                # 인도가 아직 붙기 전이라 인도가 있으면 한 칸 당겨서 쓴다.
                shift = 1 if "sidewalkWidth" in sidewalk else 0
                for idx, cls in (lane_allow or {}).items():
                    lines.append(f'        <lane index="{int(idx) - shift}" allow="{cls}"/>')
                # 배경차 출발/도착 확률 배율 (randomTrips --edge-param tripWeight)
                if "trip_weight" in r:
                    lines.append(f'        <param key="tripWeight" value="{r["trip_weight"]}"/>')
                lines.append("    </edge>")
            else:
                lines.append(head + "/>")
    # 교통섬 보행로: 도로 중간 횡단보도 → 섬을 가로질러 → 섬 모서리 교차로의 보행공간
    #   via: 섬 꼭짓점 등 중간점. 보행로가 옆 도로와 나란히 교차로에 들어가면
    #   netconvert가 교차로를 크게 잡으므로, 섬 끝을 거쳐 들어가게 할 때 쓴다.
    for s in splits.values():
        if not s["walk"]:
            continue
        q, t = s["xy"], nodes[s["target"]][:2]
        mids =" ".join(f"{x},{y}" for x, y in s.get("via", []))
        lines.append(f'    <edge id="{s["walk"]}" from="{s["node"]}" to="{s["target"]}" '
                     f'priority="1" numLanes="1" speed="1.39" width="2.0" allow="pedestrian" '
                     f'spreadType="center" shape="{q[0]:.2f},{q[1]:.2f} {mids} {t[0]},{t[1]}"/>')
    lines.append("</edges>")
    with open(p("highway.edg.xml"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # 3) netconvert: 연결(어느 차선이 어디로 회전 가능한지)과 신호 프로그램은 자동 추정
    net_cmd = [checkBinary("netconvert"),
               "--node-files", p("highway.nod.xml"),
               "--edge-files", p("highway.edg.xml"),
               "--offset.disable-normalization", "true",
               "--no-turnarounds.except-deadend", "true",
               "--output.street-names", "true",
               "-o", p("highway.net.xml")]
    kept = []
    if ped_on:
        # 횡단보도는 2단계로 만든다:
        #   (1) netconvert에게 모든 교차로의 횡단보도를 추정(--crossings.guess)시켜
        #       위치/폭/어느 엣지를 가로지르는지를 받아오고
        #   (2) 그중 crossing_nodes에 해당하는 것만 남겨 다시 빌드한다.
        #   신호등 교차로에 횡단보도가 있으면 보행 신호가 신호 주기에 자동으로 들어간다.
        #   walkingareas: 인도끼리 교차로 모서리에서 이어지는 보행 공간
        guess_prefix = p("highway.guess")
        subprocess.run(net_cmd + ["--crossings.guess", "true",
                                  "--crossings.guess.speed-threshold", "40",
                                  "--plain-output-prefix", guess_prefix],
                       check=True, capture_output=True)
        want = ped.get("crossing_nodes", "traffic_light")
        if want == "traffic_light":
            want = [n for n, v in nodes.items() if v[2] == "traffic_light"]
        want = set(want) | {s["node"] for s in splits.values()}
        con = ET.parse(guess_prefix + ".con.xml").getroot()
        kept = [c for c in con.findall("crossing") if c.get("node") in want]
        mid_nodes = {s["node"] for s in splits.values()}
        for c in kept:      # 신호 없는 도로 중간 횡단보도: 차가 보행자에게 양보
            if c.get("node") in mid_nodes:
                c.set("priority", "1")
        # 빼는 횡단보도: 그 교차로에서 edges 안의 도로만 가로지르는 자동 횡단보도를 지운다
        for nc in ped.get("no_crossings") or []:
            kept = [c for c in kept if not (c.get("node") == nc["node"]
                                            and set(c.get("edges").split()) <= set(nc["edges"]))]
        # 직접 지정한 횡단보도: 같은 교차로에서 그 edges에 포함되는 자동 횡단보도를 대체
        for cs in ped.get("crossings") or []:
            edges_set = set(cs["edges"])
            kept = [c for c in kept if not (c.get("node") == cs["node"]
                                            and set(c.get("edges").split()) <= edges_set)]
            el = ET.Element("crossing", node=cs["node"], edges=" ".join(cs["edges"]),
                            priority="1" if cs.get("priority", True) else "0")
            if cs.get("width"):
                el.set("width", str(cs["width"]))
            if cs.get("shape"):     # 횡단보도 위치를 직접 지정: 양 끝 (및 중간) 좌표
                el.set("shape", " ".join(f"{px},{py}" for px, py in cs["shape"]))
            kept.append(el)
        for ext in (".nod.xml", ".edg.xml", ".con.xml", ".tll.xml", ".typ.xml", ".netccfg"):
            if os.path.exists(guess_prefix + ext):
                os.remove(guess_prefix + ext)
        net_cmd += ["--walkingareas", "true"]
    # 차선 연결 직접 지정 (from 엣지, from 차선, to 엣지, to 차선). 차선 번호는 인도 포함.
    connections = osm.get("connections") or []
    # 유턴 금지 막다른 끝: 골목 끝에서 되돌아 나오는 경로(본선 유턴 대용)를 막는다
    no_uturn = [(rid if r["to"] == n else "-" + rid, "-" + rid if r["to"] == n else rid)
                for n in osm.get("no_uturn_nodes") or []
                for rid, r in roads.items()
                if n in (r["from"], r["to"]) and min(r["lanes"]) > 0]   # 일방통행은 유턴 없음
    if kept or connections or no_uturn:
        with open(p("highway.con.xml"), "w", encoding="utf-8") as f:
            f.write("<connections>\n")
            for fr, fl, to, tl in connections:
                f.write(f'    <connection from="{fr}" to="{to}" fromLane="{fl}" toLane="{tl}"/>\n')
            for fr, to in no_uturn:
                f.write(f'    <delete from="{fr}" to="{to}"/>\n')
            for c in kept:
                f.write("    " + ET.tostring(c, encoding="unicode").strip() + "\n")
            f.write("</connections>\n")
        net_cmd += ["--connection-files", p("highway.con.xml")]
    subprocess.run(net_cmd, check=True, capture_output=True)

    # 4) 차종 — 실제 도로망은 회전이 많아 배경차 차선변경을 끌 수 없다
    vtypes_block, trip_type = _vtypes(traffic, allow_lane_keep=False)
    with open(p("highway.rou.xml"), "w", encoding="utf-8") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
</routes>
""")

    # 5) 배경차 trip: 도로망 전체에서 무작위 출발/도착 (경로는 최단경로로 자동 계산)
    t = osm.get("traffic", {})
    vph = float(t.get("vehs_per_hour", 900))
    # out_dir 안에서 상대경로로 실행: 경로가 XML 주석 헤더에 그대로 들어가는데,
    # 경로에 "--"가 있으면 주석이 깨져 duarouter가 trip 파일을 못 읽는다.
    subprocess.run(
        [sys.executable, os.path.join(os.environ["SUMO_HOME"], "tools", "randomTrips.py"),
         "-n", "highway.net.xml", "-a", "highway.rou.xml",
         "-o", "highway.trips.xml", "-r", "highway.trips.rou.xml",
         "-e", str(t.get("end", 3600)), "-p", f"{3600.0 / vph:.3f}",
         "--fringe-factor", str(t.get("fringe_factor", 5)),
         "--min-distance", str(t.get("min_distance", 300)),
         "--seed", str(t.get("seed", 42)), "--validate",
         "--edge-param", "tripWeight",
         "--trip-attributes",
         f'type="{trip_type}" departLane="best" departSpeed="{t.get("depart_speed", traffic["depart_speed"])}"'],
        check=True, capture_output=True, cwd=out_dir)
    # duarouter가 쓰인 vType 정의를 결과에 복사해 넣는데, highway.rou.xml과
    # 같이 로드하면 id 중복 오류가 나므로 trip 쪽 정의는 지운다.
    tree = ET.parse(p("highway.trips.rou.xml"))
    root = tree.getroot()
    for el in root.findall("vType") + root.findall("vTypeDistribution"):
        root.remove(el)
    if root.find("vehicle") is None:
        raise RuntimeError("randomTrips가 배경차를 하나도 만들지 못했다 "
                           "(도로망 연결이 끊겼거나 min_distance가 너무 큼)")
    tree.write(p("highway.trips.rou.xml"), encoding="utf-8", xml_declaration=True)

    # 5.5) 보행자: 인도가 있는 엣지 사이를 무작위로 걷는 사람 (경로는 SUMO가 계산,
    #      교차로는 횡단보도로 건너며 보행 신호를 지킨다)
    route_files = "highway.rou.xml,highway.trips.rou.xml"
    if ped_on and float(ped.get("persons_per_hour", 0)) > 0:
        subprocess.run(
            [sys.executable, os.path.join(os.environ["SUMO_HOME"], "tools", "randomTrips.py"),
             "-n", "highway.net.xml", "-o", "highway.peds.trips.xml",
             "-r", "highway.peds.rou.xml",
             "--pedestrians", "--prefix", "ped",
             "-e", str(t.get("end", 3600)),
             "-p", f"{3600.0 / float(ped['persons_per_hour']):.3f}",
             "--min-distance", str(ped.get("min_distance", 150)),
             "--seed", str(ped.get("seed", 7)), "--validate"],
            check=True, capture_output=True, cwd=out_dir)
        route_files += ",highway.peds.rou.xml"

    # 5.7) 이름 붙인 경로 (OSM_ROAD["routes"]) → route id "r_이름".
    #      highway.rou.xml과 분리: 그 파일은 randomTrips의 입력으로도 쓰인다.
    named_routes = osm.get("routes") or {}
    if named_routes:
        import sumolib
        net = sumolib.net.readNet(p("highway.net.xml"))
        with open(p("highway.routes.rou.xml"), "w", encoding="utf-8") as f:
            f.write("<routes>\n")
            for name, rt in named_routes.items():
                edges = rt["edges"]
                for e in edges:
                    if not net.hasEdge(e):
                        raise ValueError(f'경로 {name}: 엣지 "{e}"가 도로망에 없다')
                for a, b in zip(edges, edges[1:]):
                    if net.getEdge(b) not in net.getEdge(a).getOutgoing():
                        raise ValueError(f'경로 {name}: "{a}" → "{b}" 로 이어지지 않는다')
                f.write(f'    <route id="r_{name}" edges="{" ".join(edges)}"/>\n')
            f.write("</routes>\n")
        route_files += ",highway.routes.rou.xml"

    # 6) 배경 장식: OSM 건물/공원/물 폴리곤 (원본 osm 파일이 있을 때만)
    osm_file = src.get("osm_file")
    if osm_file and not os.path.isabs(osm_file):
        osm_file = os.path.join(base, osm_file)
    if osm_file and os.path.exists(osm_file):
        subprocess.run(
            [checkBinary("polyconvert"),
             "--osm-files", osm_file, "--net-file", p("highway.net.xml"),
             "--type-file", os.path.join(os.environ["SUMO_HOME"], "data", "typemap",
                                         "osmPolyconvert.typ.xml"),
             "-o", p("highway.scenery.xml")],
            check=True, capture_output=True)
    else:
        with open(p("highway.scenery.xml"), "w") as f:
            f.write("<additional/>\n")

    # 6.5) 교통섬 포장 (보기용 회색 다각형, 도로보다 아래층)
    additional = "highway.scenery.xml"
    islands = (ped.get("islands") or []) if ped_on else []
    if islands:
        # 섬 지정: (x, y) 한 점이면 둘러싼 영역을 자동으로 찾고,
        #          {"polygon": [(x, y), ...]} 이면 그 꼭짓점 그대로 그린다
        seeds = [s for s in islands if not isinstance(s, dict)]
        polys = (_island_polygons(p("highway.net.xml"), seeds) if seeds else []) + \
            [[tuple(q) for q in s["polygon"]] for s in islands if isinstance(s, dict)]
        with open(p("highway.islands.xml"), "w") as f:
            f.write("<additional>\n")
            for i, poly in enumerate(polys):
                shape = " ".join(f"{x:.2f},{y:.2f}" for x, y in poly)
                f.write(f'    <poly id="island_{i}" type="island" color="128,128,128" '
                        f'fill="1" layer="-0.1" shape="{shape}"/>\n')
            f.write("</additional>\n")
        additional += ",highway.islands.xml"

    # 6.7) 버스 정류장: at에 가장 가까운 지점(버스가 다닐 수 있는 차선)을 중심으로 정차 구간
    stops = osm.get("bus_stops") or {}
    if stops:
        import sumolib
        from sumolib.geomhelper import polygonOffsetWithMinimumDistanceToPoint as offset_of
        net = sumolib.net.readNet(p("highway.net.xml"))
        with open(p("highway.stops.xml"), "w", encoding="utf-8") as f:
            f.write("<additional>\n")
            for i, (name, st) in enumerate(stops.items()):
                rid = st["road"]
                cands = [l for eid in (rid, rid + ".c", "-" + rid, "-" + rid + ".c")
                         if net.hasEdge(eid) for l in net.getEdge(eid).getLanes()
                         if l.allows("bus")]
                if not cands:
                    print(f"  [버스정류장] {name}: 도로 {rid}에 버스가 다닐 차선이 없어 건너뜀")
                    continue
                lane = min(cands, key=lambda l: sumolib.geomhelper.distancePointToPolygon(
                    st["at"], l.getShape()))
                pos = offset_of(st["at"], lane.getShape())
                half_len = float(st.get("length", 25.0)) / 2.0
                start = max(0.0, pos - half_len)
                end = min(lane.getLength(), pos + half_len)
                f.write(f'    <busStop id="stop_{i}" name="{name}" lane="{lane.getID()}" '
                        f'startPos="{start:.2f}" endPos="{end:.2f}" friendlyPos="true"/>\n')
            f.write("</additional>\n")
        additional += ",highway.stops.xml"

    # 6.8) 끊김 없이 보이게 할 교차로 (보기용, 차량 동작에 영향 없음)
    seamless = osm.get("seamless") or []
    if seamless:
        with open(p("highway.seams.xml"), "w", encoding="utf-8") as f:
            f.write("<additional>\n")
            for line in _seam_polys(p("highway.net.xml"), seamless):
                f.write("    " + line + "\n")
            f.write("</additional>\n")
        additional += ",highway.seams.xml"

    # 6.9) 신호 주기 고정: netconvert는 도로 모양이 바뀔 때마다 신호를 다시 계산해서
    #      주기가 달라질 수 있다. OSM_ROAD["signals"]로 고정한다:
    #        green: [{"from": 엣지, "to": 엣지, "seconds": 초}]
    #               그 움직임이 초록인 단계 중 차량 신호가 켜지는 첫 단계를 이 시간으로
    #        cycle: 전체 주기(초). 긴 초록 단계(7초 이상)에 남는 시간을 똑같이 나눔
    signals = osm.get("signals") or {}
    if signals:
        root = ET.parse(p("highway.net.xml")).getroot()
        links, all_links = {}, {}    # (신호, from, to) → 첫 링크 번호 / 모든 차선의 링크 번호
        for c in root.findall("connection"):
            if c.get("tl"):
                key = (c.get("tl"), c.get("from"), c.get("to"))
                links.setdefault(key, int(c.get("linkIndex")))
                all_links.setdefault(key, []).append(int(c.get("linkIndex")))
        with open(p("highway.tls.xml"), "w", encoding="utf-8") as f:
            f.write("<additional>\n")
            for tl in root.findall("tlLogic"):
                sig = signals.get(tl.get("id"))
                if not sig:
                    continue
                phases = [[int(float(ph.get("duration"))), ph.get("state")]
                          for ph in tl.findall("phase")]
                for g in sig.get("green", []):
                    li = links.get((tl.get("id"), g["from"], g["to"]))
                    i = None if li is None else next(
                        (i for i, (d, s) in enumerate(phases) if s[li] in "Gg"), None)
                    if i is None:
                        raise ValueError(f'신호 {tl.get("id")}: {g["from"]} → {g["to"]} 초록 단계가 없다')
                    phases[i][0] = int(g["seconds"])
                # follow: 한 움직임이 다른 움직임과 같은 신호를 받게 함 (예: 버스차로 직진 = 본선 직진).
                # 그 결과 아무 신호도 켜지지 않는 단계(빈 단계)는 지운다.
                for fw in sig.get("follow", []):
                    li = links.get((tl.get("id"), fw["from"], fw["to"]))
                    ref = links.get((tl.get("id"), fw["like"][0], fw["like"][1]))
                    if li is None or ref is None:
                        raise ValueError(f'신호 {tl.get("id")}: follow 대상 연결이 없다: {fw}')
                    for li in all_links[(tl.get("id"), fw["from"], fw["to"])]:   # 모든 차선
                        for ph in phases:
                            ph[1] = ph[1][:li] + ph[1][ref] + ph[1][li + 1:]
                if sig.get("follow"):
                    phases = [ph for ph in phases if any(c in "Gg" for c in ph[1])
                              or (any(c in "yY" for c in ph[1]))]
                    # 노랑만 남은 단계 중 바로 앞 단계가 지워진 것도 지운다
                    kept = []
                    for ph in phases:
                        if not any(c in "Gg" for c in ph[1]) and (not kept or not any(
                                c in "Gg" for c in kept[-1][1])):
                            continue
                        kept.append(ph)
                    phases = kept
                if "cycle" in sig:
                    longs = [i for i, (d, s) in enumerate(phases)
                             if d > 6 and "G" in s and "y" not in s]
                    extra = int(sig["cycle"]) - sum(d for d, _ in phases)
                    share = extra // len(longs)
                    for k, i in enumerate(longs):
                        phases[i][0] += share + (1 if k < extra - share * len(longs) else 0)
                f.write(f'    <tlLogic id="{tl.get("id")}" type="static" '
                        f'programID="config" offset="0">\n')
                for d, s in phases:
                    f.write(f'        <phase duration="{d}" state="{s}"/>\n')
                f.write("    </tlLogic>\n")
            f.write("</additional>\n")
        additional += ",highway.tls.xml"

    # 7) GUI: 도로망 전체가 보이게 시작 (zoom 100 = 네트워크 전체)
    #    보행자는 실제 크기(0.5m)로 그리면 화면에서 거의 안 보여서,
    #    확대/축소와 무관하게 일정한 화면 크기(constantSize)로 키워 그린다.
    persons = ('\n        <persons person_exaggeration="3" person_constantSize="1"/>\n    '
               if ped_on else "")
    # 우선 도로 끝의 흰 표시 막대를 숨긴다 (신호등 빨강/초록과 양보선은 그대로 표시)
    link_rules = '\n        <edges realisticLinkRules="1"/>\n    ' if osm.get("seamless") else ""
    with open(p("highway.gui.xml"), "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world">{link_rules}{persons}</scheme>
    <viewport zoom="100" x="{(min(xs) + max(xs)) / 2:.1f}" y="{(min(ys) + max(ys)) / 2:.1f}" angle="0"/>
    <delay value="{float(SIMULATION.get("gui_delay", 100.0))}"/>
</viewsettings>
""")

    cfg_path = p("highway.sumocfg")
    with open(cfg_path, "w") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="{route_files}"/>
        <additional-files value="{additional}"/>
    </input>
    <gui_only>
        <gui-settings-file value="highway.gui.xml"/>
    </gui_only>
</configuration>
""")
    return cfg_path


def _blob(cx: float, cy: float, r: float, rng, n: int = 14,
          roughness: float = 0.35) -> str:
    """유기적인(울퉁불퉁한) 수관 실루엣 다각형을 만든다.

    실제 나무를 위에서 내려다보면 완전한 원이 아니라 잎 뭉치들이
    만드는 불규칙한 테두리다. 꼭짓점마다 반지름을 무작위로 흔들되,
    이웃 꼭짓점과 평균을 내서(smoothing) 뾰족하지 않고 부드럽게
    울퉁불퉁한 실루엣을 얻는다.
    """
    import math
    raw = [1.0 + rng.uniform(-roughness, roughness) for _ in range(n)]
    sm = [(raw[i - 1] + raw[i] + raw[(i + 1) % n]) / 3.0 for i in range(n)]
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rr = r * sm[i]
        pts.append(f"{cx + rr * math.cos(a):.2f},{cy + rr * math.sin(a):.2f}")
    return " ".join(pts)


def _write_scenery(out_dir: str, road: dict):
    """도로변 나무를 highway.scenery.xml (additional 파일)로 생성.

    항공뷰(top-down) 조경처럼 보이도록 나무 한 그루를 여러 겹으로 그린다:
      layer 1  그림자  : 수관을 남동쪽으로 살짝 밀어낸 반투명 검정 blob
                         (해가 북서쪽에 있는 항공사진 느낌)
      layer 2  수관 몸통: 중간톤 초록의 불규칙 실루엣
      layer 3  잎 뭉치  : 수관 안에 어두운/짙은 클럼프 1~2개
                         → 캐노피의 입체적인 질감
      layer 4  하이라이트: 북서쪽으로 치우친 밝은 초록 작은 blob
    배치도 실제 조경처럼: 단독목 / 2~3그루 군락 / 낮은 관목이 섞인다.
    SCENERY["trees"]가 False면 빈 additional 파일 생성.
    """
    import random
    path = os.path.join(out_dir, "highway.scenery.xml")
    cfg = SCENERY if isinstance(SCENERY, dict) else {}
    if not cfg.get("trees", False):
        with open(path, "w") as f:
            f.write("<additional/>\n")
        return

    rng = random.Random(int(cfg.get("seed", 7)))
    spacing = float(cfg.get("tree_spacing", 45.0))
    jitter = float(cfg.get("tree_jitter", 0.5))
    side = float(cfg.get("side_offset", 6.0))
    half = float(road["length"]) / 2.0
    road_half_width = 3.2 * int(road["num_lanes"])
    # 교차 도로(양방향) 폭 + 여유: 이 x 구간에는 나무를 심지 않는다
    cross_clear = (3.2 * int(road.get("cross_num_lanes", road["num_lanes"]))
                   + side + 8.0)

    # (몸통, 어두운 클럼프, 밝은 하이라이트) 초록 팔레트 — 수종별 톤
    palettes = [
        ("46,110,44", "32,84,32", "96,158,84"),
        ("38,98,40",  "26,72,28", "84,146,74"),
        ("56,122,50", "40,92,38", "110,168,92"),
        ("30,88,36",  "20,64,26", "76,138,70"),
    ]
    bush_greens = ["70,124,58", "84,136,64", "62,112,52"]

    lines = ["<additional>"]
    tid = [0]

    def draw_tree(x, y, r):
        body, dark, light = rng.choice(palettes)
        i = tid[0]; tid[0] += 1
        # 그림자 (남동쪽 +x, -y 로 오프셋, 반투명)
        lines.append(f'    <poly id="sh_{i}" color="20,26,18,70" fill="1" '
                     f'layer="1" shape="{_blob(x + r*0.35, y - r*0.35, r*1.02, rng)}"/>')
        # 수관 몸통
        lines.append(f'    <poly id="tr_{i}" color="{body}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng)}"/>')
        # 잎 뭉치 1~2개 (어두운 톤, 수관 내부 무작위 위치)
        for k in range(rng.randint(1, 2)):
            ox = rng.uniform(-0.35, 0.35) * r
            oy = rng.uniform(-0.35, 0.35) * r
            lines.append(f'    <poly id="cl_{i}_{k}" color="{dark}" fill="1" '
                         f'layer="3" shape="{_blob(x + ox, y + oy, r * rng.uniform(0.4, 0.6), rng, n=10)}"/>')
        # 하이라이트 (북서쪽 -x, +y — 그림자 반대 방향)
        lines.append(f'    <poly id="hl_{i}" color="{light}" fill="1" '
                     f'layer="4" shape="{_blob(x - r*0.28, y + r*0.28, r*0.34, rng, n=8, roughness=0.25)}"/>')

    def draw_bush(x, y):
        i = tid[0]; tid[0] += 1
        g = rng.choice(bush_greens)
        r = rng.uniform(1.0, 1.8)
        lines.append(f'    <poly id="bsh_{i}" color="20,26,18,55" fill="1" '
                     f'layer="1" shape="{_blob(x + 0.3, y - 0.3, r, rng, n=8)}"/>')
        lines.append(f'    <poly id="bs_{i}" color="{g}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng, n=8)}"/>')

    for side_sign, base_y in [(+1, road_half_width + side),
                              (-1, -road_half_width - side)]:
        x = -half + rng.uniform(0, spacing)
        while x < half:
            if abs(x) < cross_clear:          # 교차 도로 위는 건너뜀
                x += spacing
                continue
            y = base_y + side_sign * rng.uniform(0, 4.0)
            roll = rng.random()
            if roll < 0.12:
                draw_bush(x, y)                        # 15%: 낮은 관목
            elif roll < 0.67:
                draw_tree(x, y, rng.uniform(2.6, 4.6))  # 55%: 단독목 (실제 수관 스케일)
            else:
                # 25%: 2~3그루 군락 (겹치듯 붙여 심어 숲 덩어리 느낌)
                n = rng.randint(2, 4)
                for _ in range(n):
                    gx = x + rng.uniform(-5.5, 5.5)
                    gy = y + side_sign * rng.uniform(-2.0, 4.5)
                    draw_tree(gx, gy, rng.uniform(2.2, 3.8))
                x += spacing * 0.4                     # 군락 뒤엔 간격 조금 더
            x += spacing * rng.uniform(1.0 - jitter, 1.0 + jitter)
    lines.append("</additional>")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
