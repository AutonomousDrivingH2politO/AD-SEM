#!/usr/bin/env python3
import time
import sys

# Import your current Stepper class
from shared_objects.Motor import Stepper

def main():
    print("Initializing Brake Motor for testing...")
    
    try:
        # Using the exact parameters from your current working stop_node.py
        brake_motor = Stepper(
            interface="can",
            data_rate=1_000_000,
            module_id=1,  # Brake module ID
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,
            V1=100_000,
            A1=50_000,
            D1=50_000
        )
    except Exception as e:
        print(f"[ERROR] Failed to connect to motor: {e}")
        sys.exit(1)

    try:
        print("Motor initialized successfully.")
        print("Brake test starting in 3 seconds...")
        time.sleep(3)
        
        # Execute the brake sequence
        print("Executing brake...")
        brake_motor.brake()
        print("Brake sequence completed successfully.")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user. Stopping motor.")
        brake_motor.motor.stop()
    except Exception as e:
        print(f"\n[ERROR] during brake execution: {e}")
    finally:
        # Safely disconnect
        brake_motor.disconnect_motor()
        print("Motor disconnected. Test complete.")

if __name__ == "__main__":
    main()
