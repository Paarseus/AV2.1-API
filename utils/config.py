"""
Configuration Loader
"""

import yaml
from pathlib import Path

_CONFIG_DIR = Path(__file__).parent.parent / "config"


def load_config(name="default"):
    """
    Load configuration from YAML file.

    Args:
        name: Config name without extension ("default" or "local")

    Returns:
        dict: Configuration dictionary
    """
    path = _CONFIG_DIR / f"{name}.yaml"
    with open(path) as f:
        return yaml.safe_load(f)
