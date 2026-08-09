"""PID gain persistence — load/save gains.json with atomic writes."""

from dataclasses import dataclass, asdict
import json
import os
import sys
import tempfile

GAINS_SCHEMA_VERSION = 1
DEFAULT_GAINS_PATH = "gains.json"


@dataclass
class Gains:
    """PID gains for the 3-DOF pose controller (surge/sway/yaw)."""

    surge_kp: float = 0.6
    surge_ki: float = 0.05
    surge_kd: float = 0.15
    sway_kp: float = 0.6
    sway_ki: float = 0.05
    sway_kd: float = 0.15
    yaw_kp: float = 0.8
    yaw_ki: float = 0.0
    yaw_kd: float = 0.1

    @classmethod
    def from_file(cls, path: str = DEFAULT_GAINS_PATH) -> "Gains":
        """Load from JSON. If file missing, return defaults (do not raise).

        Defensive on parse errors — returns defaults + logs warning to stderr.
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
        if version != GAINS_SCHEMA_VERSION:
            print(f"[gains] schema version mismatch in {path}: "
                  f"expected {GAINS_SCHEMA_VERSION}, got {version}. Using defaults.",
                  file=sys.stderr)
            return cls()

        # Parse per-axis dicts with fallbacks
        try:
            surge = data.get("surge", {})
            sway = data.get("sway", {})
            yaw = data.get("yaw", {})
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
        data = {
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
        """Map to PoseController constructor args.

        Surge and sway share kp/ki/kd (matches existing PoseController signature).
        """
        return {
            "kp": self.surge_kp,
            "ki": self.surge_ki,
            "kd": self.surge_kd,
            "yaw_kp": self.yaw_kp,
            "yaw_ki": self.yaw_ki,
            "yaw_kd": self.yaw_kd,
        }

    def to_dict(self) -> dict:
        """Return a JSON-serializable dict matching gains.json schema."""
        return {
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
