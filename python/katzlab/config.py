"""Configuration loading.

Two files drive the whole system:

``system.yaml``      machine limits, link settings, safety timings
``controller.yaml``  which physical axis/button is which logical control,
                     produced by ``scripts/calibrate_controller.py``

Both are plain YAML so they can be edited without touching code, which is the
point of moving off the .ino constants in the first place.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
SYSTEM_CONFIG = CONFIG_DIR / "system.yaml"
CONTROLLER_CONFIG = CONFIG_DIR / "controller.yaml"


class ConfigError(RuntimeError):
    """Raised when a config file is missing, malformed, or internally inconsistent."""


# --------------------------------------------------------------------------
# system.yaml
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LinkConfig:
    mode: str = "single"
    baud: int = 115200
    port: str = "auto"
    motion_port: str = "auto"
    laser_port: str = "auto"
    connect_timeout_s: float = 8.0
    read_timeout_s: float = 6.0

    def __post_init__(self) -> None:
        if self.mode not in ("single", "split"):
            raise ConfigError(f"link.mode must be 'single' or 'split', got {self.mode!r}")


@dataclass(frozen=True)
class InputConfig:
    source: str = "mac_gamepad"
    poll_hz: float = 60.0
    deadzone: float = 0.04
    expo: float = 0.35
    # Per-axis overrides of `expo`. Higher = more of the stick's travel is
    # spent in the slow end, so the same wrist movement changes the axis less.
    axis_expo: dict[str, float] = field(default_factory=dict)
    # Per-axis overrides of `deadzone`. Only useful where an axis is used for
    # fine positioning and its stick does not drift.
    axis_deadzone: dict[str, float] = field(default_factory=dict)
    # Axes that are physically one stick, shaped as a vector rather than
    # independently. Each entry: {"axes": [a, b], "deadzone": f, "expo": f}.
    paired_axes: list[dict] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.source not in ("mac_gamepad", "teensy_host"):
            raise ConfigError(
                f"input.source must be 'mac_gamepad' or 'teensy_host', got {self.source!r}"
            )
        if not 0.0 <= self.deadzone < 1.0:
            raise ConfigError("input.deadzone must be in [0, 1)")
        if not 0.0 <= self.expo <= 1.0:
            raise ConfigError("input.expo must be in [0, 1]")
        for axis, value in self.axis_expo.items():
            if not 0.0 <= value <= 1.0:
                raise ConfigError(f"input.axis_expo.{axis} must be in [0, 1]")
        for axis, value in self.axis_deadzone.items():
            if not 0.0 <= value < 1.0:
                raise ConfigError(f"input.axis_deadzone.{axis} must be in [0, 1)")

    def pair_for(self, axis: str) -> dict | None:
        """The stick this axis belongs to, if it is half of one."""
        for pair in self.paired_axes:
            if axis in pair.get("axes", ()):
                return pair
        return None

    def expo_for(self, axis: str) -> float:
        return self.axis_expo.get(axis, self.expo)

    def deadzone_for(self, axis: str) -> float:
        return self.axis_deadzone.get(axis, self.deadzone)


@dataclass(frozen=True)
class LinearConfig:
    max_rate_mm_s: float = 2.5
    min_mm: float = 0.0
    max_mm: float = 177.0
    max_chunk_mm: float = 0.50


@dataclass(frozen=True)
class RotationConfig:
    max_rate_deg_s: float = 20.0
    max_chunk_deg: float = 5.0


@dataclass(frozen=True)
class FlexionConfig:
    max_rate_deg_s: float = 8.0
    command_deadband_deg: float = 0.3
    firmware_deadband_deg: float = 0.15
    min_deg: float = -270.0
    max_deg: float = 270.0
    caution_deg: float = 150.0
    warning_deg: float = 200.0
    major_deg: float = 250.0


@dataclass(frozen=True)
class MotionConfig:
    # "velocity" = Motion_Velocity_Server.ino, continuous and smooth.
    # "chunked"  = Combined_Motion_Laser.ino, one blocking move per command.
    protocol: str = "velocity"
    loop_hz: float = 20.0
    step_delay_us: int = 200

    def __post_init__(self) -> None:
        if self.protocol not in ("velocity", "chunked"):
            raise ConfigError(
                f"motion.protocol must be 'velocity' or 'chunked', got {self.protocol!r}"
            )
    linear: LinearConfig = field(default_factory=LinearConfig)
    rotation: RotationConfig = field(default_factory=RotationConfig)
    flexion: FlexionConfig = field(default_factory=FlexionConfig)


@dataclass(frozen=True)
class LaserConfig:
    fire_mode: str = "momentary"
    min_fire_duration_s: float = 0.6
    max_fire_duration_s: float = 10.0
    ready_edge_open_s: float = 3.0
    standby_drives_ready: bool = False
    estop_drives_ready: bool = True
    fire_ch1_invert: bool = False
    fire_ch2_invert: bool = False
    require_ready_before_fire: bool = True
    arm_dwell_s: float = 0.5
    release_dwell_s: float = 0.4

    def __post_init__(self) -> None:
        if self.max_fire_duration_s <= self.min_fire_duration_s:
            raise ConfigError(
                "laser.max_fire_duration_s must exceed min_fire_duration_s"
            )
        if self.fire_mode not in ("momentary", "latched"):
            raise ConfigError(
                f"laser.fire_mode must be 'momentary' or 'latched', got {self.fire_mode!r}"
            )


@dataclass(frozen=True)
class SafetyConfig:
    watchdog_timeout_s: float = 0.5
    estop_on_controller_loss: bool = True
    start_disarmed: bool = True


@dataclass(frozen=True)
class LoggingConfig:
    level: str = "INFO"
    session_log_dir: str = "logs"
    record_motion: bool = True
    record_hz: float = 20.0


@dataclass(frozen=True)
class SystemConfig:
    link: LinkConfig = field(default_factory=LinkConfig)
    input: InputConfig = field(default_factory=InputConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    laser: LaserConfig = field(default_factory=LaserConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key) or {}
    if not isinstance(value, dict):
        raise ConfigError(f"section {key!r} must be a mapping, got {type(value).__name__}")
    return value


def load_system_config(path: Path | str | None = None) -> SystemConfig:
    """Read system.yaml. Missing keys fall back to the dataclass defaults."""
    path = Path(path) if path else SYSTEM_CONFIG
    if not path.exists():
        raise ConfigError(f"system config not found: {path}")

    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        raise ConfigError(f"{path} must contain a YAML mapping at the top level")

    motion_raw = _section(raw, "motion")
    motion = MotionConfig(
        protocol=str(motion_raw.get("protocol", "velocity")),
        loop_hz=float(motion_raw.get("loop_hz", 20.0)),
        step_delay_us=int(motion_raw.get("step_delay_us", 200)),
        linear=LinearConfig(**_section(motion_raw, "linear")),
        rotation=RotationConfig(**_section(motion_raw, "rotation")),
        flexion=FlexionConfig(**_section(motion_raw, "flexion")),
    )

    return SystemConfig(
        link=LinkConfig(**_section(raw, "link")),
        input=InputConfig(**_section(raw, "input")),
        motion=motion,
        laser=LaserConfig(**_section(raw, "laser")),
        safety=SafetyConfig(**_section(raw, "safety")),
        logging=LoggingConfig(**_section(raw, "logging")),
    )


# --------------------------------------------------------------------------
# controller.yaml
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Binding:
    """One logical control bound to one physical input.

    ``source`` is "axis" or "button". Triggers are reported as buttons on some
    controllers and as axes on others, so both spellings have to be supported --
    that is exactly the difference the calibration wizard resolves.
    """

    source: str
    index: int
    invert: bool = False
    threshold: float = 0.5

    def __post_init__(self) -> None:
        if self.source not in ("axis", "button"):
            raise ConfigError(f"binding source must be 'axis' or 'button', got {self.source!r}")
        if self.index < 0:
            raise ConfigError(f"binding index must be >= 0, got {self.index}")


# Logical control names used everywhere downstream. Bound to the physical layout
# described in the project brief:
#   left stick   -> linear travel
#   right stick  -> rotation (X) and flexion (Y)
#   bumpers      -> "secondary" triggers
#   triggers     -> "primary" triggers
AXIS_CONTROLS = ("linear", "rotation", "flexion")
BUTTON_CONTROLS = (
    "enable_ready",   # left secondary  (LB)
    "standby",        # left primary    (LT)
    "fire",           # right secondary (RB)
    "pause",          # right primary   (RT)
    "estop",          # X, bottom of the right-hand button cluster
    "home",           # H, right of the right-hand button cluster
)


@dataclass(frozen=True)
class ControllerConfig:
    name: str
    guid: str
    axes: dict[str, Binding]
    buttons: dict[str, Binding]
    # SDL backend this mapping was captured on ("mfi" or "iokit"). The same pad
    # enumerates with different names AND different axis/button counts on each,
    # so the mapping is only valid on the one it was made for.
    backend: str = ""

    def validate(self) -> None:
        missing_axes = [c for c in AXIS_CONTROLS if c not in self.axes]
        missing_buttons = [c for c in BUTTON_CONTROLS if c not in self.buttons]
        if missing_axes or missing_buttons:
            raise ConfigError(
                "controller.yaml is incomplete -- re-run "
                "scripts/calibrate_controller.py. Missing axes: "
                f"{missing_axes or 'none'}; missing buttons: {missing_buttons or 'none'}"
            )


def load_controller_config(path: Path | str | None = None) -> ControllerConfig:
    path = Path(path) if path else CONTROLLER_CONFIG
    if not path.exists():
        raise ConfigError(
            f"controller config not found: {path}\n"
            "Run 'python scripts/calibrate_controller.py' with the controller "
            "plugged in to generate it."
        )

    raw = yaml.safe_load(path.read_text()) or {}
    if not isinstance(raw, dict):
        raise ConfigError(f"{path} must contain a YAML mapping at the top level")

    def bindings(section: str) -> dict[str, Binding]:
        out: dict[str, Binding] = {}
        for name, spec in (raw.get(section) or {}).items():
            if not isinstance(spec, dict):
                raise ConfigError(f"{section}.{name} must be a mapping")
            out[name] = Binding(**spec)
        return out

    cfg = ControllerConfig(
        name=str(raw.get("name", "unknown")),
        guid=str(raw.get("guid", "")),
        axes=bindings("axes"),
        buttons=bindings("buttons"),
        backend=str(raw.get("backend", "")),
    )
    cfg.validate()
    return cfg


def save_controller_config(cfg: ControllerConfig, path: Path | str | None = None) -> Path:
    path = Path(path) if path else CONTROLLER_CONFIG
    path.parent.mkdir(parents=True, exist_ok=True)

    def dump(bindings: dict[str, Binding]) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for name, b in bindings.items():
            spec: dict[str, Any] = {"source": b.source, "index": b.index}
            if b.invert:
                spec["invert"] = True
            if b.source == "axis":
                spec["threshold"] = b.threshold
            out[name] = spec
        return out

    payload = {
        "name": cfg.name,
        "guid": cfg.guid,
        "backend": cfg.backend,
        "axes": dump(cfg.axes),
        "buttons": dump(cfg.buttons),
    }
    path.write_text(
        "# Generated by scripts/calibrate_controller.py -- safe to hand-edit.\n"
        + yaml.safe_dump(payload, sort_keys=False)
    )
    return path
