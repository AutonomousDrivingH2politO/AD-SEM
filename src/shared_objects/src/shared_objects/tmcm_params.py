#!/usr/bin/env python3
import gc
from pytrinamic.modules import TMCM1260


def print_tmcm1260_parameters():
    print("=" * 60)
    print("TMCM-1260 Axis Parameters (AP) and IDs")
    print("=" * 60)

    target_module_name = "pytrinamic.modules.TMCM1260"
    found_ap_class = None

    # Scan memory specifically for the TMCM1260's AP class
    for obj in gc.get_objects():
        if isinstance(obj, type) and obj.__name__ in ["AP", "APs"]:
            if hasattr(obj, "__module__") and obj.__module__ == target_module_name:
                found_ap_class = obj
                break

    if not found_ap_class:
        print(
            f"Error: Could not locate the AP class specifically for {target_module_name}."
        )
        return

    # Extract the parameters
    params = []
    for attr in dir(found_ap_class):
        if not attr.startswith("_") and not callable(getattr(found_ap_class, attr)):
            val = getattr(found_ap_class, attr)
            if isinstance(val, int):
                params.append((attr, val))

    # Sort and print them nicely
    params.sort(key=lambda x: x[0])

    print(f"{'Parameter Name':<35} | {'ID'}")
    print("-" * 45)
    for name, param_id in params:
        print(f"{name:<35} | {param_id}")

    print("=" * 60)
    print(f"Total Parameters Found: {len(params)}")


if __name__ == "__main__":
    print_tmcm1260_parameters()
