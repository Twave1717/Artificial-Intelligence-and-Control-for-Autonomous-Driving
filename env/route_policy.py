"""학습 없이 경로를 따라가는 규칙. 수집의 --policy route-follow와 scripts/validate_road.py가 쓴다.

numpy만 쓴다. 도로만 다시 만드는 사람에게 torch를 요구하지 않는다.
"""
import numpy as np


def route_following_action(observation):
    """완만하게 가속하고, 현재 차선이 경로에서 끊기면 더 멀리 이어지는 옆 차선으로 옮긴다.

    관측 [21..25]는 (좌2, 좌1, 현재, 우1, 우2) 차선의 경로 기준 연결성이다.
    """
    _left2, left1, current, right1, _right2 = (float(value) for value in observation[21:26])
    lane_change = 0.0
    if current < 1.0:
        connectivity, direction = max((left1, 1.0), (right1, -1.0))
        if connectivity > current:
            lane_change = direction
    return np.array([0.25, lane_change], dtype=np.float32)
