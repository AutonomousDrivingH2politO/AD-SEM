
from pathlib import Path
import time

import carla
import yaml

# Load LiDAR configuration
config_path = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "lidar_profiles.yaml"
)

config = yaml.safe_load(config_path.read_text())

# Connect to CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(30.0)

world = client.get_world()

if world.get_settings().synchronous_mode:
    raise RuntimeError(
        "CARLA is in synchronous mode. "
        "This script expects asynchronous mode."
    )

library = world.get_blueprint_library()

# Vehicle blueprint
vehicle_bp = library.find("vehicle.tesla.model3")
vehicle_bp.set_attribute("role_name", "hero")
vehicle_bp.set_attribute("ros_name", "ego")

vehicle = None
lidar = None
obstacle = None

try:

    # Blueprints for test scenario
    obstacle_bp = library.find("vehicle.audi.tt")
    obstacle_bp.set_attribute("role_name", "test_obstacle")

    carla_map = world.get_map()

    # Find a location where both vehicles can spawn
    for spawn_point in carla_map.get_spawn_points():

        vehicle = world.try_spawn_actor(
            vehicle_bp, spawn_point
        )

        if vehicle is None:
            continue

        waypoint = carla_map.get_waypoint(
            spawn_point.location,
            lane_type=carla.LaneType.Driving
        )

        # Try different distances along the road
        if waypoint is not None:
            for distance in (12.0, 15.0, 18.0):
                for next_wp in waypoint.next(distance):

                    target = next_wp.transform
                    target.location.z += 0.4

                    obstacle = world.try_spawn_actor(
                        obstacle_bp, target
                    )

                    if obstacle is not None:
                        print(
                            f"Obstacle placed about {distance} m ahead"
                        )
                        break

                if obstacle is not None:
                    break

        if obstacle is not None:
            break

        # This location didn't work: try another
        vehicle.destroy()
        vehicle = None

    if vehicle is None or obstacle is None:
        raise RuntimeError(
            "Could not find a valid two-vehicle scenario"
        )

    # Keep both vehicles stationary
    vehicle.apply_control(
        carla.VehicleControl(hand_brake=True)
    )

    obstacle.apply_control(
        carla.VehicleControl(hand_brake=True)
    )

    print("Test obstacle ID:", obstacle.id)




    # Configure LiDAR
    lidar_bp = library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("ros_name", "lidar")

    for key, value in config["lidar"].items():
        lidar_bp.set_attribute(key, str(value))

    lidar_bp.set_attribute("sensor_tick", "0.0")
    lidar_bp.set_attribute("dropoff_general_rate", "0.0")
    lidar_bp.set_attribute("noise_stddev", "0.0")

    # Mount position relative to vehicle
    mount = config["mount"]

    lidar_transform = carla.Transform(
        carla.Location(
            x=mount["x"],
            y=mount["y"],
            z=mount["z"]
        )
    )

    # Spawn LiDAR
    lidar = world.spawn_actor(
        lidar_bp,
        lidar_transform,
        attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )

    # Enable native ROS 2 publishing
    lidar.enable_for_ros()

    print("Vehicle ID:", vehicle.id)
    print("LiDAR ID:", lidar.id)
    print("Channels:", config["lidar"]["channels"])
    print("ROS enabled:", lidar.is_enabled_for_ros())
    print("LiDAR running. Press Ctrl+C to stop.")

    # Keep vehicle and sensor alive
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping simulation...")

finally:
    if obstacle is not None:
        obstacle.destroy()
    if lidar is not None:
        lidar.destroy()

    if vehicle is not None:
        vehicle.destroy()

    print("Actors destroyed.")
