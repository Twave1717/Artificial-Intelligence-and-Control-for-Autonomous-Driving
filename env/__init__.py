# env 패키지: 환경(문제 정의) 관련 모든 것.
#
#   road_config.py  : 팔달문 도로 선택과 차량/교통류 설정 (지도 원본은 maps/paldalmun/)
#   mdp_config.py   : MDP 정의 (관측/행동/보상)
#   road_builder.py : road_config → 실행 폴더에 SUMO 파일 생성
#   sumo_env.py     : Gymnasium 환경 (TraCI로 SUMO 조종)
#   route_policy.py : 학습 없이 경로를 따라가는 규칙 운전자 (수집 정책, 도로 검증)
#   experiment.py   : 진입점 공통 CLI 옵션과 run_config.json 기록
