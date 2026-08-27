"""
navigation/navigator.py -- High-level navigation controller for Swerve Drive.

Consumes ``Detection`` results from any vision mode and translates
them into holonomic swerve commands sent via ``SerialComm``.

The navigator supports multiple modes (``config.NAVIGATION_MODE``):
  - ``idle``    -- motors stopped; vision runs but no commands are sent.
  - ``follow``  -- move forward and strafe to keep the target centered.
  - ``avoid``   -- strafe away from the highest-confidence detection.
  - ``patrol``  -- drive forward; stop when an obstacle is detected.

Standalone test::

    python -m navigation.navigator
"""

from __future__ import annotations

from typing import Callable, List, Optional

import config
from navigation.pid import PIDController
from navigation.swerve_kinematics import SwerveKinematics
from utils.logger import get_logger

try:
    from vision.detector import Detection
except ImportError:
    # Lightweight fallback for environments without numpy / full vision stack.
    from dataclasses import dataclass, field
    from typing import Any, Dict, Tuple

    @dataclass
    class Detection:  # type: ignore[no-redef]
        """Minimal stand-in matching vision.detector.Detection's interface."""
        label: str = ""
        confidence: float = 1.0
        bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
        center: Tuple[int, int] = (0, 0)
        extra: Dict[str, Any] = field(default_factory=dict)

log = get_logger(__name__)


class Navigator:
    """Translate vision detections into swerve motor commands.

    Parameters
    ----------
    send_fn : callable(str) -> None, optional
        A function that sends a command string to the motor controller
        (typically ``SerialComm.send``).  If *None*, commands are
        computed and returned but not dispatched.
    """

    def __init__(self, send_fn: Optional[Callable[[str], None]] = None) -> None:
        self._send: Optional[Callable[[str], None]] = send_fn
        self._mode: str = config.NAVIGATION_MODE.lower()
        self.state: str = "IDLE"
        self._frame_w: int = config.CAMERA_WIDTH
        self._frame_cx: int = self._frame_w // 2
        self._tolerance: int = config.FRAME_CENTER_TOLERANCE
        self._base_speed: int = config.BASE_SPEED
        self._turn_speed: float = config.TURN_SPEED

        self._kinematics = SwerveKinematics()

        self._pid = PIDController(
            kp=config.PID_KP,
            ki=config.PID_KI,
            kd=config.PID_KD,
            output_limits=config.PID_OUTPUT_LIMITS,
        )
        self._avoid_hold_ticks: int = 0
        self._avoid_hold_dir: int = 0
        self._target_detected_ticks: int = 0
        self._initial_approach_ticks: int = 0
        self._reacquire_ticks: int = 0
        self._avoid_completed: bool = False
        self._lost_target_count: int = 0

        log.info(
            "Navigator (Swerve) ready  [mode=%s  base_speed=%d  tolerance=%dpx  "
            "frame_center=%dpx]",
            self._mode, self._base_speed, self._tolerance, self._frame_cx,
        )

    # ── public API ───────────────────────────────

    def update(self, detections: List[Detection]) -> str:
        """Compute and send a motor command based on current detections."""
        if self._mode == "idle":
            self.state = "IDLE"
            self._target_detected_ticks = 0
            self._initial_approach_ticks = 0
            self._reacquire_ticks = 0
            self._avoid_completed = False
            self._lost_target_count = 0
            log.debug("Mode is idle -- stopping")
            return self._cmd_stop()

        if self.state == "ARRIVED":
            log.info("State: ARRIVED -- Maintaining safe stop at destination")
            return self._cmd_stop()

        targets = [d for d in detections if d.label != "obstacle"]
        obstacles = [
            d for d in detections 
            if d.label == "obstacle" 
            and (d.bbox[2] >= 20 or d.bbox[3] >= 20) 
            and abs(d.center[0] - self._frame_cx) < 220
        ]
        
        # 1. Check if we have arrived at the destination target
        if targets:
            primary = max(targets, key=lambda d: d.confidence)
            w = primary.bbox[2]
            h = primary.bbox[3]
            d_est = 157.5 / max(1, w)

            # Safe arrival distance (~0.65 - 0.75m standoff from chair, corresponding to w >= 225px, d_est ~0.70m)
            if self._avoid_completed and w >= 225:
                self.state = "ARRIVED"
                self._target_detected_ticks = 0
                self._initial_approach_ticks = 0
                self._reacquire_ticks = 0
                log.info("State: ARRIVED -- Target reached safe standoff distance (w=%d px, estimated distance=%.2f m)", w, d_est)
                return self._cmd_stop()

        if self._mode in ("follow", "autonomous"):
            if targets:
                primary = max(targets, key=lambda d: d.confidence)
                self._lost_target_count = 0

                # 1. Initial target acquisition from idle or search
                if self.state in ("SEARCH", "IDLE") or self._target_detected_ticks > 0:
                    if self.state in ("SEARCH", "IDLE"):
                        self._target_detected_ticks = 6  # Hold for ~0.18s
                        self._initial_approach_ticks = 10  # Hold for ~0.30s
                    else:
                        self._target_detected_ticks -= 1
                    self.state = "TARGET_DETECTED"
                    log.info("State: TARGET_DETECTED -- Initial target acquisition at (%d, %d)", primary.center[0], primary.center[1])
                    return self._follow(primary)

                # 2. Obstacle evasion takes immediate priority when obstacle is visible
                if obstacles and not self._avoid_completed:
                    self.state = "AVOID"
                    self._target_detected_ticks = 0
                    self._initial_approach_ticks = 0
                    self._reacquire_ticks = 0
                    obs = max(obstacles, key=lambda d: d.bbox[2] * d.bbox[3])
                    log.info("State: AVOID -- Swerving around obstacle at (%d, %d)", obs.center[0], obs.center[1])
                    return self._avoid(obs, forward_progress=True)

                # 3. Initial approach towards target
                if self._initial_approach_ticks > 0:
                    self._initial_approach_ticks -= 1
                    self.state = "APPROACH"
                    log.info("State: APPROACH -- Approaching target at (%d, %d)", primary.center[0], primary.center[1])
                    return self._follow(primary)

                # 4. Reacquisition after obstacle evasion
                if self.state == "AVOID" or self._reacquire_ticks > 0:
                    if self.state == "AVOID":
                        self._reacquire_ticks = 6  # Hold for ~0.18s
                        self._avoid_completed = True
                    else:
                        self._reacquire_ticks -= 1
                    self.state = "REACQUIRE"
                    log.info("State: REACQUIRE -- Target reacquired after avoiding obstacle")
                    return self._follow(primary)

                # 5. Continuous approach towards destination target
                self.state = "APPROACH"
                return self._follow(primary)

            else:
                if obstacles and not self._avoid_completed:
                    self.state = "AVOID"
                    obs = max(obstacles, key=lambda d: d.bbox[2] * d.bbox[3])
                    return self._avoid(obs, forward_progress=True)

                self._lost_target_count += 1
                if self._lost_target_count < 6 and self.state in ("APPROACH", "REACQUIRE", "TARGET_DETECTED"):
                    # Brief loss tolerance: maintain current trajectory
                    return self._cmd_swerve(vx=self._base_speed, vy=0, omega=0)
                
                # No target visible -> Search state (controlled in-place scan rotation)
                self.state = "SEARCH"
                self._target_detected_ticks = 0
                self._initial_approach_ticks = 0
                self._reacquire_ticks = 0
                log.debug("State: SEARCH -- Rotating in place to scan for target")
                return self._cmd_swerve(vx=0, vy=0, omega=15)

        elif self._mode == "search":
            self.state = "SEARCH"
            self._target_detected_ticks = 0
            self._initial_approach_ticks = 0
            self._reacquire_ticks = 0
            return self._cmd_swerve(vx=0, vy=0, omega=15)

        elif self._mode == "avoid":
            if obstacles:
                obs = max(obstacles, key=lambda d: d.bbox[2] * d.bbox[3])
                return self._avoid(obs, forward_progress=True)
            elif targets:
                primary = max(targets, key=lambda d: d.confidence)
                return self._follow(primary)
            else:
                return self._cmd_swerve(vx=self._base_speed, vy=0, omega=0)

        return self._cmd_stop()

    def set_mode(self, mode: str) -> None:
        """Change navigation mode at runtime."""
        old_mode = self._mode
        self._mode = mode.lower()
        self._pid.reset()
        if self._mode == "idle":
            self.state = "IDLE"
            self._target_detected_ticks = 0
            self._initial_approach_ticks = 0
            self._reacquire_ticks = 0
            self._avoid_completed = False
            self._lost_target_count = 0
        log.info(
            "Navigation mode changed: '%s' -> '%s'", old_mode, self._mode,
        )


    # ── strategies ───────────────────────────────

    def _follow(self, det: Detection) -> str:
        """Move forward and strafe to center the target using PID correction."""
        error = det.center[0] - self._frame_cx
        correction = self._pid.compute(error)

        # If we recently evaded an obstacle, hold lateral clearance past it
        if self._avoid_hold_ticks > 0:
            self._avoid_hold_ticks -= 1
            if self._avoid_hold_dir > 0 and correction < 0:
                correction = 0.0
            elif self._avoid_hold_dir < 0 and correction > 0:
                correction = 0.0

        log.debug(
            "follow: error=%+dpx  correction=%.1f  tolerance=%dpx",
            error, correction, self._tolerance,
        )

        if abs(error) <= self._tolerance:
            # Target is centered: Drive straight forward at base speed
            return self._cmd_swerve(vx=self._base_speed, vy=0, omega=0)
        else:
            # Swerve: Strafe (vy) to center while rotating heading (omega) to align with target
            w_cmd = int(correction * 0.15)
            return self._cmd_swerve(vx=self._base_speed, vy=correction, omega=w_cmd)

    def _avoid(self, det: Detection, forward_progress: bool = True) -> str:
        """Strafe away from the obstacle centre with safe lateral clearance."""
        error = det.center[0] - self._frame_cx
        vx_cmd = int(self._base_speed * 0.15) if forward_progress else 0
        vy_cmd = int(self._base_speed * 2.20)

        log.debug("avoid: error=%+dpx  tolerance=%dpx  forward=%s", error, self._tolerance, forward_progress)

        if error <= 0:
            # Obstacle is on left or center: Swerve RIGHT (+Y evasion in Navigator cmd)
            self._avoid_hold_ticks = 25
            self._avoid_hold_dir = +1
            return self._cmd_swerve(vx=vx_cmd, vy=vy_cmd, omega=0)
        else:
            # Obstacle is on right: Swerve LEFT (-Y evasion in Navigator cmd)
            self._avoid_hold_ticks = 25
            self._avoid_hold_dir = -1
            return self._cmd_swerve(vx=vx_cmd, vy=-vy_cmd, omega=0)


    def _patrol(self, detections: List[Detection]) -> str:
        """Drive forward; stop when something is too close."""
        for det in detections:
            _, _, w, _ = det.bbox
            if w > self._frame_w * 0.40:
                log.debug(
                    "patrol: obstacle '%s' is too close (bbox_w=%d > %.0fpx)",
                    det.label, w, self._frame_w * 0.40,
                )
                return self._cmd_stop()

        log.debug("patrol: path clear -- driving forward at %d", self._base_speed)
        return self._cmd_swerve(vx=self._base_speed, vy=0, omega=0)

    # ── command helpers ──────────────────────────

    def _send_cmd(self, cmd: str) -> str:
        """Dispatch *cmd* to the motor controller and return it."""
        if self._send is not None:
            self._send(cmd)
            log.debug("Sent command: %s", cmd)
        else:
            log.debug("Command (no send_fn): %s", cmd)
        return cmd

    def _cmd_swerve(self, vx: float, vy: float, omega: float) -> str:
        """Calculate swerve angles and speeds and generate the command string."""
        wheels = self._kinematics.calculate(vx, vy, omega)
        # Format: SWERVE:a_fr,s_fr,a_fl,s_fl,a_rl,s_rl,a_rr,s_rr
        parts = []
        for angle, speed in wheels:
            parts.extend([f"{angle:.1f}", f"{speed:.1f}"])
        cmd_str = "SWERVE:" + ",".join(parts)
        return self._send_cmd(cmd_str)

    def _cmd_stop(self) -> str:
        return self._cmd_swerve(0, 0, 0)


# ── standalone test ──────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  Navigator (Swerve) -- standalone test suite")
    print("=" * 60)

    passed = 0
    failed = 0

    def _assert(cond: bool, tag: str) -> None:
        global passed, failed
        if cond:
            passed += 1
            print(f"  [PASS] {tag}")
        else:
            failed += 1
            print(f"  [FAIL] {tag}")

    # Helper: collect sent commands
    commands: list[str] = []

    def _capture(cmd: str) -> None:
        commands.append(cmd)

    # ── 1. Idle mode stops ───────────────────────
    print("\n[1] Idle mode always sends STOP (all zeros)")
    commands.clear()
    nav = Navigator(send_fn=_capture)
    nav.set_mode("idle")
    det = Detection(label="person", confidence=0.9, bbox=(400, 200, 50, 50), center=(425, 225))
    cmd = nav.update([det])
    _assert(cmd.startswith("SWERVE:0.0,0.0"), f"idle + detection -> '{cmd}'")

    # ── 2. Follow mode strafes ───────────────────
    print("\n[2] Follow mode strafes to center detection")
    nav.set_mode("follow")
    nav._pid.reset()
    
    # Target right of center (cx=425, frame_cx=320). Should strafe right.
    # A right strafe means angles should be around 90 deg for all wheels.
    cmd = nav.update([det])
    parts = cmd.split(":")[1].split(",")
    fr_angle = float(parts[0])
    _assert(fr_angle > 0, f"follows object to the right (angle={fr_angle}) -> '{cmd}'")

    # ── summary ──────────────────────────────────
    total = passed + failed
    print("\n" + "=" * 60)
    print(f"  Results: {passed}/{total} passed", end="")
    if failed:
        print(f"  ({failed} FAILED)")
    else:
        print("  -- all clear!")
    print("=" * 60)
