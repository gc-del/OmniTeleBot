from dataclasses import dataclass, field
from pathlib import Path
import yaml
from typing import Any, Dict


def get_config_data() -> Dict[str, Any]:
    """Load `config.yaml` from package if present, else return empty dict."""
    here = Path(__file__).resolve().parent
    cfg_path = here / "config.yaml"
    if cfg_path.exists():
        try:
            with cfg_path.open('r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}
    return {}


@dataclass
class TelegripConfig:
    """Lightweight config holder for telegrip defaults.

    Attributes are accessed as attributes; unknown keys fall back to the
    loaded YAML contents via `get_config_data()`.
    """
    _data: Dict[str, Any] = field(default_factory=get_config_data)

    def __getattr__(self, item: str) -> Any:
        if item in self._data:
            return self._data[item]
        raise AttributeError(item)

    def as_dict(self) -> Dict[str, Any]:
        return dict(self._data)
