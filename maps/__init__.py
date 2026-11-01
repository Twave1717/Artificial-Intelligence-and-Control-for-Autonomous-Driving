"""팔달문 도로 원본(map.json + 검수한 native 도로망)을 읽는다.

도로를 고르는 일과 실행 파일을 만드는 일을 나눈다. 실행 파일은 env/road_builder.py가
만들고, 도로망과 map.json은 scripts/build_road.py가 만든다.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

MAPS_DIR = Path(__file__).resolve().parent
FOLDER = MAPS_DIR / "paldalmun"


def load_map() -> dict:
    """map.json을 매번 새 dict로 읽고, 실행망이 map.json이 검수한 그 파일인지 확인한다."""
    definition = json.loads((FOLDER / "map.json").read_text(encoding="utf-8"))
    source = definition["network_source"]
    net_path = FOLDER / source["path"]
    if hashlib.sha256(net_path.read_bytes()).hexdigest() != source["sha256"]:
        raise ValueError(
            f"{net_path.name}의 sha256이 map.json과 다릅니다. 흔한 원인은 줄바꿈 변환입니다. Windows의 Git은 "
            "*.xml을 CRLF로 받을 수 있으니 LF 그대로 다시 받으세요. 도로를 바꿨다면 scripts/build_road.py를 다시 실행하세요.")
    definition["net_path"] = str(net_path)
    definition["context_path"] = str(FOLDER / "source" / "context.add.xml")
    return definition
