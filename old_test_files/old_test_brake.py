#!/usr/bin/env python3
import time
import os
import sys

# ─────────────────────────────────────────────────────────────
# Add ONLY the path where Motor.py is located
# ─────────────────────────────────────────────────────────────

# Correct import: not from shared_objects, but directly from Motor
from shared_objects.Motor import Stepper

# Optional: verify the file loaded
import inspect

print("Stepper definition from:", inspect.getfile(Stepper))

# Module IDs
modules = {"steering": 3, "brake": 1}

# Create instance with positional arguments
brake_motor = Stepper(
    interface="can",
    data_rate=1000000,
    module_id=1,  # modulo dello sterzo
    max_velocity=80_000,
    max_acc=50_000,
    MaxDeceleration=50_000,
    V1=100_000,
    A1=50_000,
    D1=50_000,
)

# Execute brake
if __name__ == "__main__":
    try:
        print("Brake test starts in 3 seconds…")
        time.sleep(3)
        brake_motor.brake()
        print("Brake sequence completed successfully.")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        brake_motor.disconnect_motor()
        print("Motor disconnected. Test complete.")

