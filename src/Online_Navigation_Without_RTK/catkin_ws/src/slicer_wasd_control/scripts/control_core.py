"""兼容原速度复用器的配置加载；不含PWM换算或新增速度过滤。"""
from pathlib import Path
import yaml
DEFAULT_CONFIG=Path(__file__).resolve().parents[1]/'config/teleop_params.yaml'
def load_config(path=DEFAULT_CONFIG):
    return yaml.safe_load(Path(path).read_text())
