"""PID gain persistence — load/save gains.json with atomic writes."""

from dataclasses import dataclass, asdict, fields
import json
import os
import sys
import tempfile

GAINS_SCHEMA_VERSION = 2
DEFAULT_GAINS_PATH = "gains.json"

# Default damper parameters
DEFAULT_DAMPER_KV = 0.5
DEFAULT_DAMPER_VEL_LEAK = 0.5
DEFAULT_DAMPER_ACCEL_LPF_HZ = 5.0
DEFAULT_DAMPER_ACCEL_DEADBAND = 0.05


@dataclass
class Gains:
    """PID gains for the 3-DOF pose controller (surge/sway/yaw) + velocity damper."""

    surge_kp: float = 0.6
    surge_ki: float = 0.05
    surge_kd: float = 0.15
    sway_kp: float = 0.6
    sway_ki: float = 0.05
    sway_kd: float = 0.15
    yaw_kp: float = 0.8
    yaw_ki: float = 0.0
    yaw_kd: float = 0.1

    # Velocity damper (added in schema v2; disabled if all defaults)
    damper_enabled: bool = False
    damper_kv: float = DEFAULT_DAMPER_KV
    damper_vel_leak: float = DEFAULT_DAMPER_VEL_LEAK
    damper_accel_lpf_hz: float = DEFAULT_DAMPER_ACCEL_LPF_HZ
    damper_accel_deadband: float = DEFAULT_DAMPER_ACCEL_DEADBAND

    @classmethod
    def from_file(cls, path: str = DEFAULT_GAINS_PATH) -> "Gains":
        """Load from JSON. If file missing, return defaults (do not raise).

        Accepts both v1 and v2 schemas. v1 files (no velocity_damper section)
        load with damper disabled. Defensive on parse errors.
        """
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except FileNotFoundError:
            print(f"[gains] {path} not found, using defaults", file=sys.stderr)
            return cls()
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[gains] failed to read {path}: {exc}, using defaults",
                  file=sys.stderr)
            return cls()

        # Validate version
        version = data.get("version")
        if version not in (1, GAINS_SCHEMA_VERSION):
            print(f"[gains] schema version mismatch in {path}: "
                  f"expected 1 or {GAINS_SCHEMA_VERSION}, got {version}. Using defaults.",
                  file=sys.stderr)
            return cls()

        # Parse per-axis dicts with fallbacks
        try:
            surge = data.get("surge", {})
            sway = data.get("sway", {})
            yaw = data.get("yaw", {})

            # Parse velocity_damper section (v2)
            damper = data.get("velocity_damper", {})
            damper_enabled = bool(damper.get("enabled", False))

            return cls(
                surge_kp=float(surge.get("kp", 0.6)),
                surge_ki=float(surge.get("ki", 0.05)),
                surge_kd=float(surge.get("kd", 0.15)),
                sway_kp=float(sway.get("kp", 0.6)),
                sway_ki=float(sway.get("ki", 0.05)),
                sway_kd=float(sway.get("kd", 0.15)),
                yaw_kp=float(yaw.get("kp", 0.8)),
                yaw_ki=float(yaw.get("ki", 0.0)),
                yaw_kd=float(yaw.get("kd", 0.1)),
                damper_enabled=damper_enabled,
                damper_kv=float(damper.get("kv", DEFAULT_DAMPER_KV)),
                damper_vel_leak=float(damper.get("vel_leak", DEFAULT_DAMPER_VEL_LEAK)),
                damper_accel_lpf_hz=float(damper.get("accel_lpf_hz", DEFAULT_DAMPER_ACCEL_LPF_HZ)),
                damper_accel_deadband=float(damper.get("accel_deadband", DEFAULT_DAMPER_ACCEL_DEADBAND)),
            )
        except (TypeError, ValueError, KeyError) as exc:
            print(f"[gains] malformed data in {path}: {exc}, using defaults",
                  file=sys.stderr)
            return cls()

    def to_file(self, path: str = DEFAULT_GAINS_PATH) -> None:
        """Save to JSON with schema version + pretty indent.

        Atomic write: write to a temp file in the same directory, fsync,
        then rename over the target.
        """
        data = self.to_dict()

        # Atomic write: tmp file in same dir → fsync → rename
        directory = os.path.dirname(path) or "."
        tmp_path = None
        try:
            fd, tmp_path = tempfile.mkstemp(
                suffix=".tmp", prefix=".gains_", dir=directory
            )
            with os.fdopen(fd, "w") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, path)
            tmp_path = None  # renamed successfully, no cleanup needed
        except Exception as exc:
            # Clean up temp file on failure
            if tmp_path is not None:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
            raise exc

    def to_pose_controller_kwargs(self) -> dict:
        """Map to PoseController constructor/update_gains args.

        Surge and sway are independent, explicit gains -- neither is
        derived from the other (PoseController takes surge_kp/ki/kd and
        sway_kp/ki/kd as separate, equally-required arguments).
        """
        return {
            "surge_kp": self.surge_kp,
            "surge_ki": self.surge_ki,
            "surge_kd": self.surge_kd,
            "sway_kp": self.sway_kp,
            "sway_ki": self.sway_ki,
            "sway_kd": self.sway_kd,
            "yaw_kp": self.yaw_kp,
            "yaw_ki": self.yaw_ki,
            "yaw_kd": self.yaw_kd,
        }

    def to_dict(self) -> dict:
        """Return a JSON-serializable dict matching gains.json schema (v2)."""
        result = {
            "version": GAINS_SCHEMA_VERSION,
            "surge": {
                "kp": self.surge_kp,
                "ki": self.surge_ki,
                "kd": self.surge_kd,
            },
            "sway": {
                "kp": self.sway_kp,
                "ki": self.sway_ki,
                "kd": self.sway_kd,
            },
            "yaw": {
                "kp": self.yaw_kp,
                "ki": self.yaw_ki,
                "kd": self.yaw_kd,
            },
        }
        if self.damper_enabled:
            result["velocity_damper"] = {
                "enabled": self.damper_enabled,
                "kv": self.damper_kv,
                "vel_leak": self.damper_vel_leak,
                "accel_lpf_hz": self.damper_accel_lpf_hz,
                "accel_deadband": self.damper_accel_deadband,
            }
        return result
