"""팔달문 도로의 차량·교통 설정.

도로 형상과 경로·수요는 maps/paldalmun/map.json, 차량 특성은 이 파일,
교통량 배율은 scenario가 맡는다. 실행 진입점이 configure_map()을 호출해
세 dict를 제자리 갱신하므로 `from env.road_config import ROAD` 참조가 유지된다.
단위: 거리 m, 속도 m/s, 가속도 m/s², 교통량 대/시간.
"""
from copy import deepcopy

from maps import load_map

# baseline 자체가 혼잡한 수요다. busy는 내부 차량이 더 늘어나는 범위에서 1.25배를 쓴다.
SCENARIOS = {"quiet": 0.5, "baseline": 1.0, "busy": 1.25}

# RL 에이전트가 조종하는 승용차. 가감속 한계는 수업 원본 값이고,
# 실제 한 스텝의 가감속은 mdp_config의 행동 정의가 결정한다.
_EGO = {
    "accel": 5.4,          # 최대 가속도 (m/s²) → 행동 a=+1
    "decel": 5.4,          # 최대 감속도 (m/s²) → 행동 a=-1
    "max_speed": 33.33,    # 차량 자체의 최고속도. 경로 제한속도로 다시 제한한다.
    # 제한속도 배율을 seed로 뽑지 않는다. ego의 최고속도는 seed와 무관하다. SUMO는 speedFactor 하나만
    # 주면 그 값을 평균으로 쓰고 기본 편차 0.1로 여전히 뽑으므로 speedDev=0이 있어야 고정된다.
    "speed_factor": 1, "speed_dev": 0,
    "depart_lane": "best", # 진입부 차선 수가 경로마다 다르므로 SUMO가 연결 차선을 고른다.
    "depart_speed": 0.0,   # 정지 출발. 혼잡한 진입부에서 빈틈이 작아도 투입된다.
    # 신호 없는 도로에서 교차로 우선권과 안전 차간은 SUMO 보조(31)가 지킨다.
    # 512는 자율 차선변경을 끄고 TraCI 명령의 안전 간격만 검사한다.
    "speed_mode": 31, "lane_change_mode": 512,
}

# 배경차는 추종모델 네 종류를 섞어 특정 모델의 버릇에 과적합되지 않게 한다.
# IDM/EIDM은 선호 최소간격을 수 mm 침범할 수 있는 근사 모델이라 0.1초 간격과
# tau=1.8초를 쓴다. 현장 운전행태 실측값은 아니다.
_CONTROLLERS = {
    "krauss": {"probability": .35, "carFollowModel": "Krauss",
               "accel": 2.0, "decel": 4.5, "tau": 1.2,
               "min_gap": 2.5, "sigma": .3, "speed_dev": .08, "color": "1,1,0"},
    "idm": {"probability": .30, "carFollowModel": "IDM",
            "accel": 1.8, "decel": 4.5, "tau": 1.8,
            "min_gap": 2.5, "speed_dev": .08, "color": "0,1,1"},
    "eidm": {"probability": .20, "carFollowModel": "EIDM",
             "accel": 2.0, "decel": 4.5, "tau": 1.8,
             "min_gap": 2.5, "speed_dev": .08, "color": "1,.6,0"},
    "acc": {"probability": .15, "carFollowModel": "ACC",
            "accel": 2.0, "decel": 4.5, "tau": 1.5,
            # 기본 +2m/s 여유 대신 Krauss의 안전 추종속도를 상한으로 써서
            # 짧은 합류부의 ACC 후미 추돌을 막는다.
            "collision_avoidance_override": 0.0,
            "min_gap": 2.5, "speed_dev": .05, "color": "1,0,1"},
}

ROAD, EGO, TRAFFIC = {}, {}, {}


def configure_map(scenario="baseline", seed=0, ego_route=None):
    """교통 조건과 ego 경로를 고르고 ROAD, EGO, TRAFFIC을 제자리 갱신한다.

    ego_route가 None이면 map.json의 ego_route다. 잘못된 값이면 아무것도 바꾸지 않는다.
    seed는 SUMO와 환경의 초기 난수에 전달한다. 지도 자체는 바뀌지 않으므로
    같은 도로·같은 교통 조건에서 PPO와 BC를 비교할 수 있다.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"알 수 없는 scenario: {scenario}; 선택: {tuple(SCENARIOS)}")
    # SUMO의 CLI seed는 signed int이다.
    if not isinstance(seed, int) or seed < 0 or seed >= 2**31:
        raise ValueError("seed는 0 이상 2**31 미만의 정수여야 합니다.")
    network = load_map()
    route_id = network["ego_route"] if ego_route is None else ego_route
    if route_id not in network["routes"]:
        raise ValueError(f"알 수 없는 ego 경로: {route_id!r}; 선택: {tuple(network['routes'])}")
    facts = network["route_design"][route_id]
    ROAD.clear()
    ROAD.update(map_id=network["id"], map_version=network["version"], name=network["name"],
                network=network, length=facts["length_m"], num_lanes=facts["first_edge_lanes"],
                speed_limit=facts["max_speed_mps"], ego_route=route_id, scenario=scenario, seed=seed)
    EGO.clear()
    EGO.update(deepcopy(_EGO), max_speed=min(facts["max_speed_mps"], _EGO["max_speed"]), route_id=route_id)
    TRAFFIC.clear()
    TRAFFIC.update(
        # 배경차의 물리적 최고속도. 차로 제한속도는 SUMO가 따로 지킨다.
        # 출발 속도는 그 자리에서 안전한 최대값. 고정값이면 교차로 바로 앞 골목 진입로에서 투입하지 못한다.
        max_speed=network["max_lane_speed_mps"], depart_speed="max",
        # 횡방향 속도뿐 아니라 가속도도 제한하는 SL2015를 사용한다.
        # 원형 도로의 순간 횡점프와 저속에서 옆으로 미끄러지는 현상을 줄인다.
        lateral_resolution=0.8,
        lateral_model={"accel": 0.8, "max_speed": 1.0, "standing_speed": 0.0,
                       "speed_factor": 0.2, "roundabout_cooperation": 0.2},
        # -1은 SUMO의 모델별 기본값(Krauss=1, IDM/EIDM/ACC=0.1)이다.
        # minGap 전체를 강제하면 몸체가 2.49m 떨어진 정지 차량도 삭제된다.
        collision_min_gap_factor=-1.0,
        demand_scale=SCENARIOS[scenario],
        vehs_per_hour=sum(float(f["vehs_per_hour"]) for f in network["flows"]) * SCENARIOS[scenario],
        controllers=deepcopy(_CONTROLLERS),
    )
