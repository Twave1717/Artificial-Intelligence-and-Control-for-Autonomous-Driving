# 합정역 도로

합정역 일대 실제 도로를 SUMO로 옮긴 도로망이다. 도로망과 부속 파일은 서예인이 만들었고(`합정역` 브랜치 `ec6fbf6`), 팔달문과 같은 `maps/<지도 id>/` 구조로 옮겼다.

![합정역 일대 지도(왼쪽)와 같은 범위에 겹친 SUMO 도로망(오른쪽, 붉은색)](references/hapjeong_road.png)

## 보기

```bash
python view_road.py --map hapjeong
python view_road.py --map hapjeong --scenario busy
```

## 지금 상태

- 도로를 보고 배경 교통을 돌릴 수 있다. 학습(`train.py`, `test.py`, `collect_pomdp_data.py`)에는 아직 쓸 수 없다. 일반 도로 95개 중 79개의 0번 차로가 인도라서, 차로 번호를 관측으로 쓰는 학습 환경이 틀린 값을 읽는다. 학습 환경은 이 지도를 받으면 이유를 보여 주고 멈춘다(`map.json`의 `trainable`).
- 신호 14곳, 인도·횡단보도, 보행자 601명, 버스 전용 차로가 있다. 배경 차량은 팔달문과 같은 규칙으로 경계 진입로 9곳에서 들어온다(baseline 시간당 1,320대).
- 배경 검증(`references/stability_validation.json`, seed 0·7·42 × quiet·baseline·busy, 1,800초)은 통과하지 못했다. 9번 중 8번에서 충돌이 났고(모두 13번), busy seed 7에서 순간이동이 4번 났다. 충돌 13번 중 대부분은 횡단보도에서 정체로 멈춘 차와 보행자가 부딪힌 것이고, 교차로 `J58`에서 차끼리 부딪힌 경우도 있다.

## 파일

| 파일 | 내용 |
|---|---|
| `map.json` | ego 경로, 배경 교통, 부속 파일 목록, 학습 가능 여부. `scripts/build_hapjeong.py`가 만든다 |
| `source/network.net.xml` | 실행 도로망 |
| `source/signals.add.xml` | 고정 신호 프로그램 |
| `source/islands.add.xml`, `source/seams.add.xml` | 교통섬과 차로 이음선 표시 |
| `source/stops.add.xml` | 버스 정류장 2곳 |
| `source/pedestrians.rou.xml` | 보행자 경로 |
| `source/context.add.xml` | 건물·녹지 외곽선. 화면 표시용 |
| `source/area.osm.xml` | OSM 원자료 (© OpenStreetMap contributors, ODbL 1.0) |
| `generator/` | 도로망을 만든 원본 생성기(`osm_builder.py`와 그때의 `road_builder.py`, `road_config.py`). 수업 원본 코드 구조를 전제로 해서 이 저장소에서는 그대로 실행되지 않는다 |
| `references/stability_validation.json` | `python scripts/validate_road.py --map hapjeong --out maps/hapjeong/references/stability_validation.json`의 결과 |

도로망을 바꾸면 `source/`의 파일을 바꾼 뒤 `python scripts/build_hapjeong.py`로 `map.json`을 다시 만든다.
