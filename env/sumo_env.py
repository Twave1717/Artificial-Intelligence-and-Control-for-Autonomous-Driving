"""SUMO 도시 도로 RL 환경 — Gymnasium 인터페이스 + TraCI.

┌─────────────────────────────────────────────────────────────┐
│  강화학습 루프에서 이 클래스의 역할                              │
│                                                             │
│    obs, _ = env.reset()          # 에피소드 시작, 초기 관측     │
│    obs, r, term, trunc, _ = env.step(action)  # 0.5초 진행    │
│                                                             │
│  내부적으로는 TraCI(Traffic Control Interface)라는 소켓 API로  │
│  SUMO 시뮬레이터를 원격 조종한다:                               │
│    traci.simulationStep()  → 시뮬레이션을 1스텝 진행            │
│    traci.vehicle.setSpeed() → 차량 속도 명령                   │
│    traci.vehicle.getSpeed() 등 → 상태 조회                     │
└─────────────────────────────────────────────────────────────┘

관측/행동/보상 설정은 env/mdp_config.py 에 있다(31차원 관측, 2차원 행동).
교차 접근 차량은 아직 관측에 없으므로 도시 주행의 완전한 MDP는 아니다.
교차로 우선권과 차간거리는 SUMO 보조가 지킨다.
"""
import os
import sys

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# SUMO 설치 폴더의 tools/ 안에 traci, sumolib 파이썬 모듈이 들어있다.
# pip로도 설치했지만, 버전 불일치를 피하려면 SUMO_HOME 쪽을 우선 경로에 추가.
#
# SUMO_HOME이 설정 안 돼 있으면 OS별 기본 설치 경로를 순서대로 탐색해서
# 자동으로 잡아준다 (환경변수 설정을 깜빡해도 동작하도록).
if "SUMO_HOME" not in os.environ:
    _candidates = [
        # macOS (Homebrew) — Apple Silicon / Intel
        "/opt/homebrew/opt/sumo/share/sumo",
        "/usr/local/opt/sumo/share/sumo",
        # Linux (apt)
        "/usr/share/sumo",
        # Windows (기본 인스톨러 경로)
        r"C:\Program Files (x86)\Eclipse\Sumo",
        r"C:\Program Files\Eclipse\Sumo",
    ]
    for _p in _candidates:
        if os.path.isdir(_p):
            os.environ["SUMO_HOME"] = _p
            print(f"[sumo_env] SUMO_HOME 자동 감지: {_p}")
            break
    else:
        raise RuntimeError(
            "SUMO_HOME 환경변수가 없고 기본 경로에서도 SUMO를 찾지 못했습니다.\n"
            "  Windows: 인스톨러 재설치(SUMO_HOME 체크) 또는 환경변수 수동 설정\n"
            "  macOS  : README.md의 '설치' 절 참고\n"
            "  Linux  : sudo apt install sumo sumo-tools 후 "
            "export SUMO_HOME=/usr/share/sumo")

sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci
from sumolib import checkBinary


class SumoHighwayEnv(gym.Env):
    # 클래스 이름은 수업 원본과 같다. 생성자는 원본과 달리 traffic=을 받지 않는다.
    # 배경 교통은 road_builder가 만든 SUMO 경로 파일이 정한다.
    metadata = {"render_modes": ["human"]}

    def __init__(self, cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                 gui=False, gui_autostart=True):
        """
        Args:
            cfg_path: road_builder.build()가 만든 .sumocfg 경로
            road:     env.road_config.ROAD
            ego:      env.road_config.EGO (가속/감속 한계가 여기서 옴)
            mdp_sim:  env.mdp_config.SIMULATION
            mdp_obs:  env.mdp_config.OBSERVATION
            action:   env.mdp_config.ACTION
            reward:   env.mdp_config.REWARD
            gui:      True면 sumo-gui(화면 있는 버전)로 실행
            gui_autostart: (gui=True일 때만 의미)
                True  → 창이 뜨면 시뮬레이션이 자동으로 시작
                False → 사용자가 ▶(플레이) 버튼을 누를 때까지 대기.
                        sumo-gui는 --start 옵션이 없으면 TraCI의
                        simulationStep() 호출도 플레이 전까지 블록되므로,
                        에피소드 전체가 "▶를 눌러야 시작"되는 효과가 난다.
        """
        super().__init__()
        self.cfg_path = cfg_path
        self.gui = gui
        self.gui_autostart = gui_autostart
        self.ego_route = ego["route_id"]
        self.base_seed = int(road["seed"])
        self._seed_initialized = False
        self.ego_speed_mode = int(ego["speed_mode"])
        self.ego_lane_change_mode = int(ego["lane_change_mode"])
        self._lane_geometry = {}
        self._segment_cache = {}

        # ----- 도로/차량 설정에서 필요한 값들 -----
        self.vmax = float(road["speed_limit"])   # 속도 정규화 기준
        self.ego_depart_lane = str(ego["depart_lane"])
        self.ego_depart_speed = str(ego["depart_speed"])
        # ---- 차선변경 안전 게이트 설정 (mdp_config.ACTION["lane_change_safety"]) ----
        lcs = action["lane_change_safety"]
        self.lc_safety_enabled = bool(lcs["enabled"])
        self.lc_front_min = float(lcs["min_front_gap"])     # m
        self.lc_rear_min = float(lcs["min_rear_gap"])       # m
        self.lc_front_tau = float(lcs["front_tau"])         # s
        self.lc_rear_tau = float(lcs["rear_tau"])           # s
        self.lc_spawn_protect = float(lcs["spawn_protect"]) # m
        self.lc_horizon = float(lcs["approach_horizon"])    # s
        # 행동 a ∈ [-1,1] 을 실제 가속도로 변환할 때 쓰는 스케일:
        #   a = +1 → +max_accel (풀 액셀),  a = -1 → -max_decel (풀 브레이크)
        self.max_accel = float(ego["accel"])   # m/s^2
        self.max_decel = float(ego["decel"])   # m/s^2 (양수로 저장, 부호는 계산 시)

        # ----- MDP 설정 -----
        self.step_length = float(mdp_sim["step_length"])
        self.decision_interval = float(mdp_sim["decision_interval"])
        # 행동 한 번을 SUMO 계산 몇 번 동안 유지하는지. 결정 시각이 계산 시각과 맞아야 한다.
        self.substeps = round(self.decision_interval / self.step_length)
        if self.substeps < 1 or abs(self.substeps * self.step_length
                                    - self.decision_interval) > 1e-9:
            raise ValueError(
                f"decision_interval({self.decision_interval:g}초)은 "
                f"step_length({self.step_length:g}초)의 정수배여야 합니다.")
        self.max_steps = int(mdp_sim["max_steps"])
        self.warmup_steps = int(mdp_sim["warmup_steps"])
        self.ego_insert_timeout = float(mdp_sim["ego_insert_timeout"])
        self.visibility = float(mdp_obs["visibility"])   # 관측 거리 W
        # 밀도 정규화용 "차량 1대가 점유하는 길이" (차체 + 최소 간격)
        self.density_veh_len = float(mdp_obs.get("density_veh_length", 7.5))
        self.act_dim = int(action["dim"])
        self.accel_index = int(action.get("accel_index", 0))
        self.lane_change_index = int(action.get("lane_change_index", 1))
        self.lane_change_threshold = float(action.get("lane_change_threshold", 0.5))
        self.lane_change_duration = float(action.get("lane_change_duration", 1.0))
        self.rw = reward

        # GUI는 ego를 따라가며 가까이 보이도록 기본값을 둔다.
        self.gui_zoom = float(mdp_sim["gui_zoom"])
        self.gui_delay = float(mdp_sim["gui_delay"])
        self.gui_track_ego = bool(mdp_sim["gui_track_ego"])

        self.ego_id = "ego"        # SUMO 안에서 ego 차량을 식별하는 ID
        self.step_count = 0
        self._started = False      # SUMO 프로세스가 떠 있는지 여부

        # Gymnasium 행동 공간:
        #   action[0] = 종방향 가감속 raw ∈ [-1,1]
        #   action[1] = 차선변경 raw ∈ [-1,1], 환경 내부에서 {-1,0,+1}로 양자화
        # PPO의 Gaussian 정책과 호환하기 위해 Box를 유지한다. 요청한 차선변경은
        # info["lane_change"], 그 명령의 실행 여부는 info["lane_change_applied"](수업 원본과
        # 같은 뜻), 같은 edge에서 확인된 실제 이동은 info["lane_change_actual"]로 구분한다.
        self.action_space = spaces.Box(low=-1.0, high=1.0,
                                       shape=(self.act_dim,), dtype=np.float32)
        # 관측: 31차원, 범위 [-1,1]. 레이아웃은 env/mdp_config.py 주석 참고.
        #   [0] 내 속도, [1..20] 5개 차선 × (선행/후행 상대거리·상대속도),
        #   [21..25] 경로 기준 연결성, [26..30] 전방 밀도.
        #   시야 차선: 좌2, 좌1, 현재, 우1, 우2 (5개) — 3차선 도로에서
        #   맨 왼쪽에 있어도 오프셋 -2로 맨 오른쪽 차선까지 보인다.
        self.obs_offsets = (+2, +1, 0, -1, -2)
        n_lanes_view = len(self.obs_offsets)
        self.obs_dim = 1 + n_lanes_view * 4 + n_lanes_view + n_lanes_view
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(self.obs_dim,),
                                            dtype=np.float32)

    # ==================================================================
    # SUMO 프로세스 시작
    # ==================================================================
    def _start_sumo(self):
        # gui=True면 화면이 있는 sumo-gui, 아니면 화면 없는 sumo (학습용, 훨씬 빠름)
        binary = checkBinary("sumo-gui" if self.gui else "sumo")
        # 계산 간격과 충돌 처리(remove, 교차로 포함)는 scenario.sumocfg에 있다.
        cmd = [
            binary, "-c", self.cfg_path,
            "--no-warnings", "true",   # 콘솔 경고 끄기
            "--no-step-log", "true",   # 스텝 로그 끄기
            # Gymnasium의 RNG에서 뽑아 reset(seed)의 재현성과
            # reset(None)의 연속적인 새 교통 패턴을 함께 보장한다.
            "--seed", str(self.sumo_seed),
        ]
        if self.gui:
            cmd += ["--quit-on-end"]      # 에피소드 종료 시 창 정리
            cmd += ["--delay", str(self.gui_delay)]
            if self.gui_autostart:
                cmd += ["--start"]        # 자동 재생
            # --start가 없으면: 창이 뜬 뒤 ▶(플레이)를 눌러야 진행된다.
            #   매 에피소드가 reset()에서 SUMO를 새로 띄우므로,
            #   에피소드마다 ▶를 눌러 시작하게 된다.
        traci.start(cmd)   # SUMO를 자식 프로세스로 띄우고 소켓 연결
        self._started = True

    # ==================================================================
    # 에피소드 초기화
    # ==================================================================
    def reset(self, seed=None, options=None):
        # 첫 reset()만 지도 설정의 seed를 사용한다. 이후 reset(None)은
        # RNG를 되감지 않아 학습 에피소드마다 같은 교통이 반복되지 않는다.
        if seed is None and not self._seed_initialized:
            seed = self.base_seed
        super().reset(seed=seed)
        self._seed_initialized = True
        self.sumo_seed = int(self.np_random.integers(0, 2**31 - 1))
        self._lane_geometry.clear()
        self._segment_cache.clear()

        # SUMO 상태를 완전히 초기화하는 가장 확실한 방법은 껐다 켜는 것.
        if self._started:
            traci.close()
            self._started = False
        self._start_sumo()

        # 배경 flow를 먼저 흘려 교차로 대기열이 생긴 뒤에 ego를 투입한다.
        for _ in range(self.warmup_steps):
            traci.simulationStep()

        # ego 차량 투입 — 반드시 "도로 시작점(0m)"에서 출발
        traci.vehicle.add(self.ego_id, self.ego_route, typeID="ego_type",
                          departLane=self.ego_depart_lane,
                          departPos="0",
                          departSpeed=self.ego_depart_speed)

        # 도시 맵에서는 우선권·차간거리 보조를 기본으로 사용한다.
        # 보조 없는 자율주행 성능으로 해석하면 안 된다.
        # laneChangeMode=512는 자율 변경을 끄되 TraCI 명령의 안전간격을 지킨다.
        traci.vehicle.setSpeedMode(self.ego_id, self.ego_speed_mode)
        traci.vehicle.setLaneChangeMode(self.ego_id, self.ego_lane_change_mode)

        traci.simulationStep()  # ego가 실제로 도로에 나타나도록 1스텝 진행

        # ── ego 삽입 보증 ──
        # 도로 입구는 flow가 차를 스폰하는 자리라, 시작 차선 입구가 잠시
        # 막혀 있으면 SUMO가 삽입을 "지연"시킨다. 그 경우 ego 없이
        # 에피소드가 진행되어 즉시 종료되는 버그가 되므로, 삽입될 때까지
        # 설정한 물리 시간만큼 기다린다. 계산 간격을 바꿔도 대기 한도는 같다.
        # 혼잡하다는 이유로 차량을 강제 삽입하거나 주변 차량을 삭제하지 않는다.
        _wait = 0
        wait_limit = int(np.ceil(self.ego_insert_timeout / self.step_length))
        while self.ego_id not in traci.vehicle.getIDList() and _wait < wait_limit:
            traci.simulationStep()
            _wait += 1
        if self.ego_id not in traci.vehicle.getIDList():
            raise RuntimeError(
                f"ego 삽입 실패: 도로 입구가 {self.ego_insert_timeout:g}초 이상 막혀 있습니다. "
                "진입부 대기열과 수요 경로를 확인하세요.")

        # GUI 기본 시점: ego를 화면 중앙에서 추적하고 충분히 가까이 확대한다.
        if self.gui and self.gui_track_ego:
            try:
                view_ids = traci.gui.getIDList()
                if view_ids:
                    # tracking을 먼저 켜고 zoom을 마지막에 적용해야 SUMO가
                    # 차량 추적을 시작하면서 viewport를 다시 덮어쓰지 않는다.
                    traci.gui.trackVehicle(view_ids[0], self.ego_id)
                    traci.gui.setZoom(view_ids[0], self.gui_zoom)
            except traci.TraCIException:
                pass

        self.step_count = 0
        return self._get_obs(), {"sumo_seed": self.sumo_seed,
                                 "ego_insert_wait_seconds": (1 + _wait) * self.step_length,
                                 "background_vehicles_at_reset": len(traci.vehicle.getIDList()) - 1}

    # ==================================================================
    # 관측 만들기 (레이아웃은 env/mdp_config.py 주석 참고)
    #
    # 핵심 설계: 특정 도로에 대한 하드코딩 없이, 매 스텝 SUMO 네트워크를
    # 실시간 조회해서 "지금 내 주변"을 계산한다. 지도별 도로 이름을 이
    # 코드에 넣지 않지만, 교차 접근 교통을 모두 관측한다는 뜻은 아니다.
    #
    # 사용하는 TraCI API:
    #   getLeader / getFollower : 같은 차선의 앞/뒤 차량 (edge 경계도 넘어 탐색)
    #   getNeighbors(mode)      : 옆 차선의 앞/뒤 차량.
    #       mode 비트: bit0 = 0:왼쪽 / 1:오른쪽,  bit1 = 0:후행 / 1:선행
    #   lane.getLinks           : 차선이 다음 edge로 이어지는지 (연결성 판단)
    # ==================================================================

    # (차선 오프셋, 선행차 getNeighbors mode, 후행차 mode)
    #   오프셋 +1 = 왼쪽 차선 (SUMO는 차선 index가 왼쪽으로 갈수록 커짐)
    _NEIGHBOR_MODES = {+1: (0b10, 0b00),   # 왼쪽: 선행=2, 후행=0
                       -1: (0b11, 0b01)}   # 오른쪽: 선행=3, 후행=1

    def _offset_lane(self, offset):
        """현재 위치에서 비교 가능한 차선만 반환한다.

        교차로 내부의 서로 다른 연결 차로는 '옆 차선'이 아닐 수 있다.
        이때 현재 진행 차로만 유효하게 두고, 교차 접근 교통은 후속 관측
        버전에서 별도로 추가한다. 내부 구간을 빈 도로로 처리하지는 않는다.
        """
        edge = traci.vehicle.getRoadID(self.ego_id)
        if edge.startswith(":"):
            return traci.vehicle.getLaneID(self.ego_id) if offset == 0 else None
        index = traci.vehicle.getLaneIndex(self.ego_id) + offset
        if 0 <= index < traci.edge.getLaneNumber(edge):
            return f"{edge}_{index}"
        return None

    def _geometry(self, lane):
        """변하지 않는 길이·연결만 캐시한다."""
        if lane not in self._lane_geometry:
            # getLinks: (다음 일반 lane, 우선권, 개방, 충돌차, 내부 lane, ...)
            links = tuple((link[0], link[4]) for link in traci.lane.getLinks(lane))
            self._lane_geometry[lane] = (traci.lane.getLength(lane), links)
        return self._lane_geometry[lane]

    def _forward_segments(self, offset):
        """시야 안 경로를 (lane, 시작위치, 끝위치, ego로부터 거리)로 분해.

        '첫 번째 연결'은 좌회전·직진이 섞인 도시 교차로에서 잘못된 길을
        가리킬 수 있다. ego route의 다음 edge로 가는 연결만 따라간다.
        내부 차로도 거리와 차량 수에 포함한다. 여러 유효 연결이 있으면
        현재 차선 번호와 가까운 연결을 고른다(완전한 경로 계획은 아님).
        """
        if offset in self._segment_cache:
            return self._segment_cache[offset]
        lane = self._offset_lane(offset)
        segments = []
        if lane is None:
            return segments
        route = traci.vehicle.getRoute(self.ego_id)
        route_index = traci.vehicle.getRouteIndex(self.ego_id)
        position = traci.vehicle.getLanePosition(self.ego_id)
        distance = 0.0
        visited = set()
        while lane and distance < self.visibility and (lane, route_index) not in visited:
            visited.add((lane, route_index))
            length, links = self._geometry(lane)
            start = min(max(position, 0.0), length)
            end = min(length, start + self.visibility - distance)
            segments.append((lane, start, end, distance))
            distance += end - start
            if distance >= self.visibility or route_index + 1 >= len(route):
                break
            target_edge = route[route_index + 1]
            candidates = [link for link in links
                          if traci.lane.getEdgeID(link[0]) == target_edge]
            if not candidates:
                break
            current_index = int(lane.rsplit("_", 1)[1])
            next_lane, via = min(candidates, key=lambda link:
                                 (abs(int(link[0].rsplit("_", 1)[1]) - current_index),
                                  link[0]))
            lane = via or next_lane
            if not lane.startswith(":"):
                route_index += 1
            position = 0.0
        self._segment_cache[offset] = segments
        return segments

    def _neighbor(self, offset: int, leader: bool):
        """선행/후행의 (간격 m, 속도)를 반환. 현재 차선은 내부 연결도 탐색.

        SUMO getFollower는 합류부에서 가장 위험한 후행차를 선택한다.
        옆 차선은 SUMO 이웃 캐시와 직접 스캔 중 가까운 차량을 사용한다.
        """
        if offset == 0:
            res = (traci.vehicle.getLeader(self.ego_id, self.visibility) if leader
                   else traci.vehicle.getFollower(self.ego_id, self.visibility))
            if not res or not res[0]:
                return None
            vid, gap = res
        else:
            if self._offset_lane(offset) is None:
                return None
            front, rear = self._scan_lane_neighbors(offset)
            scan = front if leader else rear
            pairs = ()
            if offset in self._NEIGHBOR_MODES:
                modes = self._NEIGHBOR_MODES[offset]
                pairs = traci.vehicle.getNeighbors(self.ego_id, modes[0 if leader else 1])
            if not pairs:
                if scan is None or scan[0] > self.visibility:
                    return None
                return max(0.0, float(scan[0])), float(scan[1])
            vid, gap = min(pairs, key=lambda pair: pair[1])
            if scan is not None and scan[0] < gap and scan[0] <= self.visibility:
                return max(0.0, float(scan[0])), float(scan[1])
        if gap > self.visibility:
            return None
        # 음수 gap을 '차 없음'으로 바꾸면 겹침/매우 가까운 차량이 사라진다.
        return max(0.0, float(gap)), traci.vehicle.getSpeed(vid)

    def _lane_connectivity(self, offset: int) -> float:
        """경로 기준 연결성: 없음=-1, 시야 끝까지 이어짐=1, 나머지는 거리/W.

        도시 교차로에서는 '어딘가로 연결됨'보다 '지정 경로로 이어짐'이
        중요하다. 다른 방향으로만 나가는 차선은 연결된 차선으로 세지 않는다.
        """
        try:
            if self._offset_lane(offset) is None:
                return -1.0
            usable = sum(end - start for _, start, end, _ in
                         self._forward_segments(offset))
            return float(np.clip(usable / self.visibility, 0.0, 1.0))
        except traci.TraCIException:
            return -1.0  # 조회 불가를 개방된 차선으로 표시하지 않음

    def _lane_density(self, offset: int) -> float:
        """경로상 시야 내 차량 수 / (사용 가능 거리 / 차량 점유 길이).

        내부 차로와 여러 짧은 edge를 함께 센다. 차선이 없거나 지정 경로로
        이어지지 않아 사용 가능한 길이가 0이면 1(진입 불가)로 표시한다.
        """
        try:
            if self._offset_lane(offset) is None:
                return 1.0
            segments = self._forward_segments(offset)
            usable = sum(end - start for _, start, end, _ in segments)
            if usable <= 0:
                return 1.0
            vehicles = set()
            for lane, start, end, _ in segments:
                for vid in traci.lane.getLastStepVehicleIDs(lane):
                    if vid != self.ego_id and start <= traci.vehicle.getLanePosition(vid) <= end:
                        vehicles.add(vid)
            max_count = usable / self.density_veh_len
            return float(np.clip(len(vehicles) / max_count, 0.0, 1.0))
        except traci.TraCIException:
            return 1.0

    def get_detected_vehicles(self):
        """관측 대상 차선의 차량 좌표(분석/데이터셋용)를 반환한다.

        전방은 관측과 같은 경로 구간을 따라 내부 차로까지 기록한다.
        후방은 현재 edge의 차선과 SUMO가 찾은 현재 경로의 후행차를 기록한다.
        교차 접근로 전체를 감지하는 센서 모델은 아직 포함하지 않는다.
        rel_pos는 ego 앞끝 기준 경로상 앞끝 거리(m, 전방 +)이다.
        """
        detected = {}
        if self.ego_id not in traci.vehicle.getIDList():
            return []

        def record(vid, rel, offset):
            if vid == self.ego_id or abs(rel) > self.visibility:
                return
            x, y = traci.vehicle.getPosition(vid)
            detected.setdefault(vid, {"id": vid, "x": float(x), "y": float(y),
                                      "rel_pos": float(rel),
                                      "speed": float(traci.vehicle.getSpeed(vid)),
                                      "lane_offset": int(offset)})

        try:
            ego_pos = traci.vehicle.getLanePosition(self.ego_id)
            for offset in self.obs_offsets:
                lane = self._offset_lane(offset)
                if lane is None:
                    continue
                for vid in traci.lane.getLastStepVehicleIDs(lane):
                    rel = traci.vehicle.getLanePosition(vid) - ego_pos
                    if rel < 0:
                        record(vid, rel, offset)
                for segment_lane, start, end, distance in self._forward_segments(offset):
                    for vid in traci.lane.getLastStepVehicleIDs(segment_lane):
                        position = traci.vehicle.getLanePosition(vid)
                        if start <= position <= end:
                            record(vid, distance + position - start, offset)
            follower = traci.vehicle.getFollower(self.ego_id, self.visibility)
            if follower and follower[0]:
                vid, gap = follower
                rel = -(gap + traci.vehicle.getMinGap(vid)
                        + traci.vehicle.getLength(self.ego_id))
                record(vid, rel, 0)
        except traci.TraCIException:
            pass
        return list(detected.values())

    def _get_obs(self):
        self._segment_cache.clear()
        # ego가 이미 사라졌다면(충돌/도착) 0벡터 반환 — 종료 직전 마지막 관측용
        if self.ego_id not in traci.vehicle.getIDList():
            return np.zeros(self.obs_dim, dtype=np.float32)

        v = traci.vehicle.getSpeed(self.ego_id)
        obs = [v / self.vmax]                      # [0] 내 절대속도

        # ----- 5개 차선(좌2→좌1→현재→우1→우2)의 선행/후행 상대거리·상대속도 -----
        for offset in self.obs_offsets:
            lead = self._neighbor(offset, leader=True)
            if lead is None:
                obs += [1.0, 1.0]                  # 없음: 멀고 + 위협 없음
            else:
                gap, lv = lead
                obs += [gap / self.visibility,
                        float(np.clip((lv - v) / self.vmax, -1.0, 1.0))]

            foll = self._neighbor(offset, leader=False)
            if foll is None:
                obs += [-1.0, -1.0]                # 없음: 멀고 + 위협 없음
            else:
                gap, fv = foll
                obs += [-gap / self.visibility,    # 후행은 음수 부호
                        float(np.clip((fv - v) / self.vmax, -1.0, 1.0))]

        # ----- 5개 차선의 연결성 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_connectivity(offset))

        # ----- 5개 차선의 전방 차량 밀도 (같은 순서) -----
        for offset in self.obs_offsets:
            obs.append(self._lane_density(offset))

        return np.clip(np.array(obs, dtype=np.float32), -1.0, 1.0)

    # ==================================================================
    # 1 스텝 진행: 행동 적용 → SUMO를 decision_interval만큼 계산 → 보상/종료 판정
    # ==================================================================
    def _quantize_lane_change(self, raw: float) -> int:
        """연속 policy 출력을 실제 차선 명령 {-1,0,+1}로 변환한다.

        +1 = 왼쪽, 0 = 유지, -1 = 오른쪽. SUMO 차선 index가 왼쪽으로
        갈수록 증가하는 규약과 동일해서 해석이 직관적이다.
        """
        raw = float(np.clip(raw, -1.0, 1.0))
        if raw >= self.lane_change_threshold:
            return 1
        if raw <= -self.lane_change_threshold:
            return -1
        return 0

    def _scan_lane_neighbors(self, direction: int):
        """목표 차선의 앞/뒤 차량을 직접 조회해 이웃 캐시의 빈틈을 보완.

        전방은 지정 경로의 내부 차로와 다음 edge까지 함께 살핀다.
        후방 스캔은 현재 edge 범위이며, 인접 차선의 SUMO 이웃 조회와 합친다.
        """
        try:
            lane = self._offset_lane(direction)
            if lane is None:
                return None, None
            my_pos = traci.vehicle.getLanePosition(self.ego_id)
            front, rear = None, None
            for vid in traci.lane.getLastStepVehicleIDs(lane):
                if vid == self.ego_id:
                    continue
                distance = traci.vehicle.getLanePosition(vid) - my_pos
                if distance < 0:
                    gap = -distance - traci.vehicle.getLength(self.ego_id)
                    if rear is None or gap < rear[0]:
                        rear = (gap, traci.vehicle.getSpeed(vid))
            for segment_lane, start, end, distance in self._forward_segments(direction):
                for vid in traci.lane.getLastStepVehicleIDs(segment_lane):
                    if vid == self.ego_id:
                        continue
                    position = traci.vehicle.getLanePosition(vid)
                    if start <= position <= end:
                        gap = distance + position - start - traci.vehicle.getLength(vid)
                        if front is None or gap < front[0]:
                            front = (gap, traci.vehicle.getSpeed(vid))
            return front, rear
        except traci.TraCIException:
            return None, None

    def _lane_change_is_safe(self, direction: int) -> bool:
        """목표 차선으로 지금 이동해도 측면충돌이 나지 않는지 검사한다.

        3중 안전 규칙:
          (0) 입구 보호구간: ego가 도로 시작 spawn_protect(m) 이내면 변경 금지.
              flow가 입구(0m)에서 차를 새로 만드는데, 생성 "직전" 차량은
              어떤 API로도 미리 볼 수 없어 원천적으로 사각지대이기 때문.
          (1) 앞: gap > min_front_gap + front_tau × 내속도
              + 접근 여유: gap > (내속도 − 앞차속도)⁺ × approach_horizon   (내가 더 빠를 때)
          (2) 뒤: gap > min_rear_gap + rear_tau × 뒤차속도
              + 접근 여유: gap > (뒤차속도 − 내속도)⁺ × approach_horizon   (뒤가 더 빠를 때)
        이웃은 getNeighbors 결과와 차선 직접 스캔 결과 중 "더 가까운 쪽"을
        쓴다 (스폰 직후 차량 사각지대 보완).
        """
        # 차선 위치는 edge마다 0으로 돌아간다. 도시의 짧은 edge마다
        # 보호구간을 다시 적용하지 않도록 누적 주행 거리를 사용한다.
        traveled = traci.vehicle.getDistance(self.ego_id)
        if traveled < self.lc_spawn_protect:
            return False

        v = traci.vehicle.getSpeed(self.ego_id)
        scan_front, scan_rear = self._scan_lane_neighbors(direction)

        def closer(a, b):
            if a is None:
                return b
            if b is None:
                return a
            return a if a[0] <= b[0] else b

        lead = closer(self._neighbor(direction, leader=True), scan_front)
        if lead is not None:
            gap, lv = lead
            if gap <= self.lc_front_min + self.lc_front_tau * v:
                return False
            if gap <= max(v - lv, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False

        foll = closer(self._neighbor(direction, leader=False), scan_rear)
        if foll is not None:
            gap, fv = foll
            if gap <= self.lc_rear_min + self.lc_rear_tau * fv:
                return False
            if gap <= max(fv - v, 0.0) * self.lc_horizon:  # 시간여유 내 접촉 위험
                return False
        return True

    def _apply_lane_change(self, lane_change: int) -> bool:
        """가능하고 안전한 차선변경 명령을 보내고, 명령 접수 여부를 반환.

        안전 게이트(_lane_change_is_safe)를 통과하지 못한 명령은 실행하지
        않고 False를 반환한다 → step()에서 "불가능한 명령"과 동일하게
        invalid_action_penalty가 부과된다. 즉 에이전트 입장에서
        "위험한 변경 = 실행도 안 되고 감점"이라, 측면충돌로 즉사하며
        배우는 대신 안전한 타이밍을 고르는 쪽으로 학습이 유도된다.
        접수 이후에도 SUMO의 안전장치가 실행을 보류할 수 있으므로,
        실제 변경 여부는 simulationStep 이후에 별도로 확인한다.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        # 유지 행동은 새 횡이동 명령이 아니다. 매 결정마다 현재 차로로 다시
        # 명령하면 몇 초에 걸쳐 진행 중인 차선변경이 중간에 취소될 수 있다.
        if lane_change == 0:
            return True
        try:
            edge = traci.vehicle.getRoadID(self.ego_id)
            if edge.startswith(":"):
                return False
            current = traci.vehicle.getLaneIndex(self.ego_id)
            n_lanes = traci.edge.getLaneNumber(edge)
            target = current + lane_change
            if target < 0 or target >= n_lanes:
                return False
            # ★ 측면충돌 방지: 실제 이동 명령(0이 아닌 변경)에만 안전 검사
            if (lane_change != 0 and self.lc_safety_enabled
                    and not self._lane_change_is_safe(lane_change)):
                return False
            traci.vehicle.changeLane(
                self.ego_id, target, self.lane_change_duration)
            return True
        except traci.TraCIException:
            return False

    def get_privileged_state(self) -> dict:
        """POMDP 데이터셋용 simulator-side state.

        agent가 실제로 받는 입력은 _get_obs()의 부분관측이고, 이 메서드는
        분석/지도신호용 privileged state만 수집한다. 정책 입력으로 쓰지 않는다.
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return {"ego_alive": False}
        try:
            x, y = traci.vehicle.getPosition(self.ego_id)
            vehicles = []
            for vid in traci.vehicle.getIDList():
                try:
                    vx, vy = traci.vehicle.getPosition(vid)
                    vehicles.append({
                        "id": vid,
                        "x": float(vx),
                        "y": float(vy),
                        "speed": float(traci.vehicle.getSpeed(vid)),
                        "angle": float(traci.vehicle.getAngle(vid)),
                        "lateral_pos": float(traci.vehicle.getLateralLanePosition(vid)),
                        "lateral_speed": float(traci.vehicle.getLateralSpeed(vid)),
                        "lane": int(traci.vehicle.getLaneIndex(vid)),
                        "lane_id": traci.vehicle.getLaneID(vid),
                        "road_id": traci.vehicle.getRoadID(vid),
                        "lane_pos": float(traci.vehicle.getLanePosition(vid)),
                    })
                except traci.TraCIException:
                    continue
            return {
                "ego_alive": True,
                "ego_x": float(x),
                "ego_y": float(y),
                "ego_speed": float(traci.vehicle.getSpeed(self.ego_id)),
                # 차로 index만으로는 진행 중인 횡이동을 복원할 수 없다.
                "ego_angle": float(traci.vehicle.getAngle(self.ego_id)),
                "ego_lateral_pos": float(traci.vehicle.getLateralLanePosition(self.ego_id)),
                "ego_lateral_speed": float(traci.vehicle.getLateralSpeed(self.ego_id)),
                "ego_lane": int(traci.vehicle.getLaneIndex(self.ego_id)),
                "ego_lane_id": traci.vehicle.getLaneID(self.ego_id),
                "ego_road_id": traci.vehicle.getRoadID(self.ego_id),
                "ego_lane_pos": float(traci.vehicle.getLanePosition(self.ego_id)),
                "vehicles": vehicles,
            }
        except traci.TraCIException:
            return {"ego_alive": False}

    def step(self, action):
        # action = [가감속 raw, 차선변경 raw]
        action_arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if action_arr.size < self.act_dim:
            raise ValueError(
                f"action은 {self.act_dim}차원이어야 합니다. 받은 shape={np.asarray(action).shape}")

        accel_raw = float(np.clip(action_arr[self.accel_index], -1.0, 1.0))
        lane_raw = float(np.clip(action_arr[self.lane_change_index], -1.0, 1.0))
        lane_change = self._quantize_lane_change(lane_raw)
        lane_change_applied = False
        lane_before = None
        edge_before = None
        requested_speed = None

        # ----- (1) 종방향 + 횡방향 행동 적용 -----
        if self.ego_id in traci.vehicle.getIDList():
            v = traci.vehicle.getSpeed(self.ego_id)
            lane_before = int(traci.vehicle.getLaneIndex(self.ego_id))
            edge_before = traci.vehicle.getRoadID(self.ego_id)

            accel_cmd = (accel_raw * self.max_accel
                         if accel_raw >= 0.0 else accel_raw * self.max_decel)
            dv = accel_cmd * self.decision_interval
            # 맵의 정규화 vmax와 현재 차선 제한속도는 다를 수 있다.
            # 교차로 내부의 낮은 허용속도도 존중하며 SUMO 안전보조를 유지한다.
            lane_limit = traci.lane.getMaxSpeed(traci.vehicle.getLaneID(self.ego_id))
            allowed = min(self.vmax, lane_limit, traci.vehicle.getAllowedSpeed(self.ego_id))
            requested_speed = float(np.clip(v + dv, 0.0, allowed))
            traci.vehicle.setSpeed(self.ego_id, requested_speed)
            lane_change_applied = self._apply_lane_change(lane_change)

        # ----- (2) 행동을 decision_interval 동안 유지: SUMO를 substeps번 계산 -----
        # 사건은 계산마다 모은다. ego 사건이 나면 남은 계산을 하지 않는다.
        collision_ids, teleporting_ids = (), ()
        for _ in range(self.substeps):
            traci.simulationStep()
            collision_ids += tuple(traci.simulation.getCollidingVehiclesIDList())
            teleporting_ids += tuple(traci.simulation.getStartingTeleportIDList())
            arrived = self.ego_id in traci.simulation.getArrivedIDList()
            alive = self.ego_id in traci.vehicle.getIDList()
            if (arrived or not alive or self.ego_id in collision_ids
                    or self.ego_id in teleporting_ids):
                break
        self.step_count += 1

        # ----- (3) 이벤트/관측 (결정 끝에서 한 번) -----
        collided = self.ego_id in collision_ids
        teleported = self.ego_id in teleporting_ids
        # SUMO는 충돌로 지운 차량도 도착 목록에 넣는다. arrived는 목적지 도착만 뜻한다.
        arrived = arrived and not collided
        lane_after = int(traci.vehicle.getLaneIndex(self.ego_id)) if alive else None
        edge_after = traci.vehicle.getRoadID(self.ego_id) if alive else None
        # edge 전환은 차선 번호 자체가 바뀔 수 있어 변경 횟수로 세면 안 된다.
        # 경계를 넘는 스텝은 확인 불가로 남기고 같은 edge의 실제 이동만 센다.
        lane_change_observable = bool(alive and edge_before == edge_after
                                      and not edge_after.startswith(":"))
        actual_lane_change = ((lane_after - lane_before) if lane_change_observable else 0)

        obs = self._get_obs()
        terminated, truncated = False, False
        ego_speed, ego_gap = None, None
        termination_reason = None

        # ----- (4) 기존 보상 유지 -----
        if collided:
            reward = -float(self.rw["collision_penalty"])
            terminated = True
            termination_reason = "collision"
        elif arrived:
            reward = float(self.rw["arrival_bonus"])
            terminated = True
            termination_reason = "arrived"
        elif teleported:
            reward = 0.0
            terminated = True
            termination_reason = "teleport"
        elif not alive:
            reward = 0.0
            terminated = True
            termination_reason = "missing"
        else:
            ego_speed = traci.vehicle.getSpeed(self.ego_id)
            same_lane_leader = self._neighbor(0, leader=True)
            if same_lane_leader is not None:
                ego_gap = same_lane_leader[0]
            reward = float(self.rw["speed_weight"]) * ego_speed / self.vmax
            if (ego_gap is not None and
                    ego_gap < float(self.rw["close_gap_threshold"])):
                reward -= float(self.rw["close_gap_penalty"])

            # 도시 기본값은 0. 교차로 대기도 '느린 앞차'로 보이므로 정당한
            # 대기를 구분하기 전까지는 정체 페널티를 끈다.
            bp = float(self.rw.get("blocked_penalty", 0.0))
            if (bp > 0.0 and ego_gap is not None
                    and ego_gap < float(self.rw.get("blocked_gap", 20.0))
                    and ego_speed < float(self.rw.get("blocked_speed_frac", 0.7))
                        * self.vmax):
                reward -= bp

            # ---- 차선변경 보상 항 ----
            # lane_change        : 이번 스텝에 policy가 낸 양자화 명령 (-1/0/+1)
            # lane_change_applied: 그 명령이 실행됐는지 (유지 명령은 항상 True)
            # actual_lane_change : 같은 edge에서 확인된 실제 차선 이동. SL2015의 차선변경은
            #                      여러 스텝에 걸치므로 감점은 명령이 아니라 실제 이동마다 준다.
            if actual_lane_change != 0:
                # 실행된 변경마다 소액 감점 → 이득 없는 지그재그 억제.
                # 변경 덕에 속도 보상이 그 이상 늘면 순이득이므로,
                # "필요할 때만 바꾸는" 정책으로 수렴하게 만드는 항이다.
                reward -= float(self.rw.get("lane_change_penalty", 0.0))
            elif lane_change != 0 and not lane_change_applied:
                # 명령은 냈는데 실행 불가(차선 없음/교차로)였던 경우 감점
                # → 의미 없는 행동 출력을 줄이는 정칙화 (rl_action_penalty 대응)
                reward -= float(self.rw.get("invalid_action_penalty", 0.0))

        if self.step_count >= self.max_steps and not terminated:
            truncated = True
            termination_reason = "time_limit"

        info = {
            "collided": collided,
            "simulation_collided": bool(collision_ids),
            "collision_ids": collision_ids,
            "teleported": teleported,
            "teleporting_ids": teleporting_ids,
            "arrived": arrived,
            "termination_reason": termination_reason,
            "sumo_seed": self.sumo_seed,
            "speed": ego_speed,
            "gap": ego_gap,
            "accel_raw": accel_raw,
            "lane_change_raw": lane_raw,
            "lane_change": lane_change,
            "lane_change_applied": lane_change_applied,
            "lane_change_actual": actual_lane_change,
            "lane_change_observable": lane_change_observable,
            "lane_before": lane_before,
            "lane_after": lane_after,
            "requested_speed": requested_speed,
            "speed_intervened": bool(alive and requested_speed is not None and
                                      abs(traci.vehicle.getSpeed(self.ego_id)
                                          - requested_speed) > 1e-3),
        }
        return obs, float(reward), terminated, truncated, info

    # ==================================================================
    def close(self):
        """환경 정리. 학습이 끝나면 반드시 호출해서 SUMO 프로세스를 종료할 것."""
        if self._started:
            traci.close()
            self._started = False
