"""지도 원본(maps/<지도 id>/map.json + 검수한 native 도로망)을 읽는다.

도로를 고르는 일과 실행 파일을 만드는 일을 나눈다. 실행 파일은 env/road_builder.py가
만들고, 도로망과 map.json은 지도마다 scripts/ 아래의 빌드 스크립트가 만든다.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

MAPS_DIR = Path(__file__).resolve().parent
DEFAULT_MAP = "paldalmun"


def available_maps() -> list[str]:
    return sorted(path.parent.name for path in MAPS_DIR.glob("*/map.json"))


def load_map(map_id: str = DEFAULT_MAP) -> dict:
    """map.json을 매번 새 dict로 읽고, 실행망이 map.json이 검수한 그 파일인지 확인한다."""
    folder = MAPS_DIR / map_id
    if not (folder / "map.json").is_file():
        raise ValueError(f"알 수 없는 지도: {map_id!r}; 선택: {tuple(available_maps())}")
    definition = json.loads((folder / "map.json").read_text(encoding="utf-8"))
    source = definition["network_source"]
    net_path = folder / source["path"]
    if hashlib.sha256(net_path.read_bytes()).hexdigest() != source["sha256"]:
        raise ValueError(
            f"{net_path.name}의 sha256이 map.json과 다릅니다. 흔한 원인은 줄바꿈 변환입니다. Windows의 Git은 "
            "*.xml을 CRLF로 받을 수 있으니 LF 그대로 다시 받으세요. 도로를 바꿨다면 그 지도의 빌드 스크립트를 다시 실행하세요.")
    definition["net_path"] = str(net_path)
    definition["context_path"] = str(folder / "source" / "context.add.xml")
    extra = definition.get("extra_files", {})
    definition["extra_additional_paths"] = [str(folder / path) for path in extra.get("additional", [])]
    definition["extra_route_paths"] = [str(folder / path) for path in extra.get("routes", [])]
    return definition
