#!/usr/bin/env python3
import time

from shared_objects.Motor import Stepper


class TestBrakeMotor(Stepper):
    """Open-loop brake: move, hold, return. No slip correction."""

    def brake(
        self,
        retract_ticks: int = -20_000,
        brake_duration: float = 5.0,
        log=None,
    ) -> None:
        """
        Args:
            retract_ticks:  Microsteps to wind the rope (sign = direction).
            brake_duration: Seconds to hold position under load.
            log:            Optional callable log(event: str, **fields) for
                            per-attempt logging (e.g. stop_node._log_csv).
        """
        def emit(event, **fields):
            if log is not None:
                log(event, **fields)

        start_pos = self.get_actual_position()
        target_pos = start_pos + retract_ticks

        print(
            f"[TestBrake] Open-loop move {start_pos} -> {target_pos}, "
            f"hold {brake_duration:.1f}s"
        )
        emit("brake_approach", actual_pos=start_pos, target_pos=target_pos)

        # Move at the configured MaxVelocity (no override) 
        self.move_to_position(target_pos)
        while not self.motor.get_position_reached():
            time.sleep(0.05)
        emit(
            "brake_hold_start",
            actual_pos=self.get_actual_position(),
            target_pos=target_pos,
        )

        # Hold: the stepper stays energized at the target position 
        time.sleep(brake_duration)
        emit(
            "brake_hold_end",
            actual_pos=self.get_actual_position(),
            target_pos=target_pos,
        )

        #Return to start 
        print(f"[TestBrake] Returning to start: {start_pos}")
        self.move_to_position(start_pos)
        while not self.motor.get_position_reached():
            time.sleep(0.05)

        print("[TestBrake] Done.")
        emit("brake_done", actual_pos=self.get_actual_position())
