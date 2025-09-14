import json
import sys
from pathlib import Path
from typing import Any, Dict

# 默认配置（用于首次运行或 JSON 缺失字段）
_DEFAULTS: Dict[str, Any] = {
    "UDP_HOST": "127.0.0.1",
    "UDP_PORT": 28777,
    "VJOY_DEVICE_ID": 3,
    "TOGGLE_RUDDER_HOTKEY": "f8",
    "TOGGLE_CYCLIC_HOTKEY": "f9",
    "TOGGLE_PAUSE_HOTKEY": "left ctrl",
    "EMA_ALPHA": 0.25,
}

def _config_path() -> Path:
    # 打包后优先读取可执行文件所在目录
    if getattr(sys, "frozen", False):
        base = Path(sys.executable).parent
    else:
        base = Path(__file__).parent
    return base / "config.json"

def _load_config() -> Dict[str, Any]:
    path = _config_path()
    if not path.exists():
        # 首次运行：写出默认文件
        try:
            path.write_text(json.dumps(_DEFAULTS, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception:
            pass
        return dict(_DEFAULTS)

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("config.json root must be an object")
    except Exception:
        # 解析失败，回退默认
        return dict(_DEFAULTS)

    # 合并默认，缺失字段用默认值
    merged = dict(_DEFAULTS)
    merged.update({k: data[k] for k in data.keys() if k in _DEFAULTS})
    return merged

def reload_config() -> None:
    """运行时重新加载配置（可在热键或命令触发）"""
    globals().update(_load_config())

# 加载并导出为模块常量（保持现有引用方式）
globals().update(_load_config())

# 类型修正（确保外部使用时类型正确）
UDP_HOST: str = str(globals()["UDP_HOST"])
UDP_PORT: int = int(globals()["UDP_PORT"])
VJOY_DEVICE_ID: int = int(globals()["VJOY_DEVICE_ID"])
TOGGLE_RUDDER_HOTKEY: str = str(globals()["TOGGLE_RUDDER_HOTKEY"])
TOGGLE_CYCLIC_HOTKEY: str = str(globals()["TOGGLE_CYCLIC_HOTKEY"])
TOGGLE_PAUSE_HOTKEY: str = str(globals()["TOGGLE_PAUSE_HOTKEY"])
EMA_ALPHA: float = float(globals()["EMA_ALPHA"])