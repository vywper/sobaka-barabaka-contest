import json
import math
import os
from pathlib import Path
import sys


FIELDS = {
    "protocol_version",
    "supported_tasks",
    "self_trained_rl",
    "preferred_speed_mps",
}
TASKS = {"T2", "T3", "T4", "T5"} # таски
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "solution.json" #/opt/participant/solution.json


def _unique_object(pairs): # запрет на дублирование ключей в json
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Дубликат JSON: {key}")
        result[key] = value
    return result

def _reject_constant(value):
    raise ValueError(f"В JSON Nan, +-inf: {value}")

def parse_manifest_json(text: str) -> dict:
    """текст - в словарь."""
    value = json.loads(
        text, object_pairs_hook=_unique_object, parse_constant=_reject_constant
    )
    if type(value) is not dict:
        raise ValueError("The ping response must be a JSON object")
    return value


def validate_manifest(data: object) -> dict:
    """
    проверка структуры манифеста решения.
    """
    if type(data) is not dict:
        raise ValueError("The manifest must be a JSON object")
    if set(data) != FIELDS:
        missing = sorted(FIELDS - set(data))
        extra = sorted(set(data) - FIELDS)
        raise ValueError(f"Unexpected manifest fields: missing={missing}, extra={extra}")
    if type(data["protocol_version"]) is not str or data["protocol_version"] != "1.0":
        raise ValueError('protocol_version must be the string "1.0"')

    tasks = data["supported_tasks"]
    if type(tasks) is not list or any(type(task) is not str or task not in TASKS for task in tasks):
        raise ValueError("supported_tasks must be a list containing only T2, T3, T4, T5")
    if len(tasks) != len(set(tasks)):
        raise ValueError("supported_tasks must not contain duplicates")
    if type(data["self_trained_rl"]) is not bool:
        raise ValueError("self_trained_rl must be a JSON boolean")

    speed = data["preferred_speed_mps"]
    if "T3" not in tasks:
        if speed is not None:
            raise ValueError("preferred_speed_mps must be null when T3 is not implemented")
    elif speed is not None:
        # Exact ranges belong to T3; T1 specifies number/null, not a speed range.
        if type(speed) not in (int, float):
            raise ValueError("preferred_speed_mps must be a JSON number or null")
        if type(speed) is float and not math.isfinite(speed):
            raise ValueError("preferred_speed_mps must be finite")
    return data


def main() -> int:
    '''
    читаем файл манифеста решения, проверяем его и выводим в stdout в виде json.
    ошибка - в stderr, код возврата 1.
    '''
    config_path = Path(os.environ.get("SOLUTION_CONFIG", str(DEFAULT_CONFIG)))
    try:
        data = validate_manifest(parse_manifest_json(config_path.read_text(encoding="utf-8")))
    except (OSError, ValueError) as exc:
        print(f"Cannot read solution manifest: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(data, ensure_ascii=True, separators=(",", ":"), allow_nan=False)) # словарь - в JSON
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
