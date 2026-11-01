# 팔달문 자율주행 도로 (SUMO)

수원 팔달문 원형 차도를 중심으로 실제 도로를 SUMO로 옮긴 팀 프로젝트용 도로다. 정조로, 행궁로, 수원천로 양쪽, 창룡대로, 중부대로와 그 사이 골목 중 팀이 지도에 표시한 도로만 남겼다. 신호는 없다. 이 도로에서 바로 도는 수업 학습 코드(PPO, BC, 데이터 수집)도 함께 있다.

- 도로 ID, 경로, 정류장: [maps/paldalmun/README.md](maps/paldalmun/README.md)
- 도로를 이렇게 고른 이유와 측정 결과: [maps/paldalmun/DESIGN.md](maps/paldalmun/DESIGN.md)

![팔달문 일대 지도(왼쪽)와 같은 범위에 겹친 SUMO 도로망(오른쪽, 붉은색)](maps/paldalmun/references/paldalmun_road.png)

## 설치

Python 3.10과 SUMO 1.27.1에서 확인했다. SUMO는 따로 설치하고 `SUMO_HOME`을 설정한다.

```bash
python -m pip install -r requirements.txt
```

도로를 읽을 때 `network.net.xml`의 sha256을 검사한다. `.gitattributes`가 XML의 줄바꿈을 바꾸지 않게 하므로 Windows에서도 git으로 받으면 된다.

## 도로 보기

```bash
python view_road.py                      # sumo-gui로 연다. ▶를 눌러 재생한다
python view_road.py --scenario busy      # 교통량: quiet, baseline(기본), busy
python view_road.py --netedit            # netedit로 도로 구조만 본다
```

## 학습, 평가, 데이터 수집

```bash
python train.py --scenario baseline --seed 42
python test.py MODEL --nogui --episodes 10
python collect_pomdp_data.py --policy route-follow --episodes 10 --out-dir data/paldalmun --name run1
python train_bc.py --data data/paldalmun/run1.npz --epochs 50
```

- ego의 기본 경로는 `r0`다. 다른 경로는 `--ego-route s_left_turn`처럼 고른다.
- ego는 0.5초마다 행동을 정하고 SUMO는 0.1초 간격으로 계산한다. 에피소드는 최대 1,000번 결정(500초)이다.
- `test.py`는 같은 폴더에 `run_config.json`이 있는 모델만 평가한다. 옵션을 주지 않으면 학습 때의 설정으로 평가한다.

수업 원본 코드(git `HEAD`)와 다르게 동작하는 부분은 다음과 같다.

- SUMO 계산 간격 0.1초. 0.5초로 계산하면 교차로와 합류부의 대기열이 달라진다.
- ego 안전 보조가 켜져 있다(`speed_mode=31`, `lane_change_mode=512`). 끌지는 팀이 정한다. 값은 `env/road_config.py`의 `_EGO`에 있다.
- ego를 넣기 전에 배경 교통을 180초 흘린다.
- `blocked_penalty`가 0이다. 교차로에서 양보하며 기다리는 것까지 벌점을 받지 않게 했다.
- 차로 유지 행동은 SUMO에 명령을 보내지 않는다. 차선 변경 벌점과 `lane_changes`는 실제 차선 이동을 센다.
- 관측의 차로 연결성은 ego 경로 방향으로 이어지는 거리만 센다.
- PPO의 GAE는 시간 초과에서도 끊는다. ego가 300초 서 있다 순간이동하면 에피소드는 `teleport`로 끝난다.

## 도로 고치기

남길 도로는 `maps/paldalmun/source/drawn_edges.json`의 도로 목록이고, ego 경로는 `scripts/build_road.py`의 `ROUTES`다. 고친 뒤 다시 만들고 검증한다. 배경 차량은 도로망 경계의 진입로에서 다른 진출로로 가며, 경로는 SUMO가 고른다.

```bash
python scripts/build_road.py
python scripts/validate_road.py --seeds 0 7 42 --scenarios quiet baseline busy --seconds 1800 --out maps/paldalmun/references/stability_validation.json
```

`build_road.py`는 검사를 모두 통과해야 `maps/paldalmun/source/network.net.xml`과 `maps/paldalmun/map.json`을 바꾼다. 같은 입력이면 같은 파일이 나온다. 가장 최근 검증에서는 45회 모두 통과했고, 배경 차량의 충돌과 순간이동은 0이었다.

## 출처

도로 자료와 README의 지도 그림은 © OpenStreetMap contributors, ODbL 1.0이다.
