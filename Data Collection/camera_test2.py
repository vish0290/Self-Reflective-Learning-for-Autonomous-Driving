import carla
import time
import os
import shutil
import math
import json

def cleanup_output_directory(output_dir):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    os.makedirs(f"{output_dir}/front")
    os.makedirs(f"{output_dir}/left")
    os.makedirs(f"{output_dir}/right")
    os.makedirs(f"{output_dir}/rear")
    print(f"Prepared output directory: {output_dir}")

# ============================================
# HELPER FUNCTIONS
# ============================================
def get_steering(vehicle, target_location):
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_yaw = vehicle_transform.rotation.yaw
    
    dx = target_location.x - vehicle_location.x
    dy = target_location.y - vehicle_location.y
    
    target_angle = math.degrees(math.atan2(dy, dx))
    error = target_angle - vehicle_yaw
    
    while error > 180: error -= 360
    while error < -180: error += 360
    
    steering = error / 30.0
    return max(-1.0, min(1.0, steering))

def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6

def get_next_waypoint(carla_map, vehicle, distance=5.0):
    vehicle_location = vehicle.get_location()
    current_waypoint = carla_map.get_waypoint(vehicle_location)
    next_waypoints = current_waypoint.next(distance)
    if next_waypoints:
        return next_waypoints[0]
    return current_waypoint

def main():
    # ============================================
    # CONNECT
    # ============================================
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    
    print(f"Connected to: {carla_map.name}")

    # ============================================
    # CLEANUP
    # ============================================
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('sensor.*'):
        actor.destroy()

    # ============================================
    # OUTPUT DIRECTORY
    # ============================================
    episode_id = "ep_001"
    output_dir = f"data/{episode_id}"
    cleanup_output_directory(output_dir)

    # ============================================
    # SPAWN VEHICLE
    # ============================================
    vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
    spawn_points = carla_map.get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print("Vehicle spawned.")

    vehicle_extent = vehicle.bounding_box.extent

    # ============================================
    # SETUP 4 CAMERAS
    # ============================================
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', '0.1')  # 10 FPS

    # Front camera
    front_transform = carla.Transform(
        carla.Location(x=vehicle_extent.x + 0.5, y=0.0, z=vehicle_extent.z),
        carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)
    )
    front_camera = world.spawn_actor(camera_bp, front_transform, attach_to=vehicle)
    front_camera.listen(lambda image: image.save_to_disk(f'{output_dir}/front/{image.frame:06d}.png'))

    # Left camera
    left_transform = carla.Transform(
        carla.Location(x=0.0, y=-vehicle_extent.y - 0.2, z=vehicle_extent.z),
        carla.Rotation(pitch=-10.0, yaw=-90.0, roll=0.0)
    )
    left_camera = world.spawn_actor(camera_bp, left_transform, attach_to=vehicle)
    left_camera.listen(lambda image: image.save_to_disk(f'{output_dir}/left/{image.frame:06d}.png'))

    # Right camera
    right_transform = carla.Transform(
        carla.Location(x=0.0, y=vehicle_extent.y + 0.2, z=vehicle_extent.z),
        carla.Rotation(pitch=-10.0, yaw=90.0, roll=0.0)
    )
    right_camera = world.spawn_actor(camera_bp, right_transform, attach_to=vehicle)
    right_camera.listen(lambda image: image.save_to_disk(f'{output_dir}/right/{image.frame:06d}.png'))

    # Rear camera
    rear_transform = carla.Transform(
        carla.Location(x=-vehicle_extent.x - 0.5, y=0.0, z=vehicle_extent.z),
        carla.Rotation(pitch=-10.0, yaw=180.0, roll=0.0)
    )
    rear_camera = world.spawn_actor(camera_bp, rear_transform, attach_to=vehicle)
    rear_camera.listen(lambda image: image.save_to_disk(f'{output_dir}/rear/{image.frame:06d}.png'))

    cameras = [front_camera, left_camera, right_camera, rear_camera]
    print("4 cameras attached: front, left, right, rear")

    # ============================================
    # DATA STORAGE
    # ============================================
    episode_data = []

    # ============================================
    # MAIN DRIVING LOOP
    # ============================================
    target_speed = 30  # km/h
    frame_count = 0
    max_frames = 500  # ~25 seconds at 20Hz

    try:
        print("Starting to drive...")
        
        while frame_count < max_frames:
            # Get next waypoint
            next_wp = get_next_waypoint(carla_map, vehicle, distance=5.0)
            target_loc = next_wp.transform.location
            
            # Calculate control
            steering = get_steering(vehicle, target_loc)
            speed = get_speed_kmh(vehicle)
            
            if speed < target_speed:
                throttle = 0.6
                brake = 0.0
            else:
                throttle = 0.0
                brake = 0.3
            
            # Apply control
            control = carla.VehicleControl()
            control.steer = steering
            control.throttle = throttle
            control.brake = brake
            vehicle.apply_control(control)
            
            # Record data every 10 frames
            if frame_count % 10 == 0:
                location = vehicle.get_location()
                rotation = vehicle.get_transform().rotation
                
                frame_data = {
                    "frame": frame_count,
                    "speed_kmh": round(speed, 2),
                    "location": {
                        "x": round(location.x, 2), 
                        "y": round(location.y, 2), 
                        "z": round(location.z, 2)
                    },
                    "rotation": {
                        "pitch": round(rotation.pitch, 2),
                        "yaw": round(rotation.yaw, 2),
                        "roll": round(rotation.roll, 2)
                    },
                    "steering": round(steering, 3),
                    "throttle": round(throttle, 2),
                    "brake": round(brake, 2)
                }
                episode_data.append(frame_data)
                print(f"Frame {frame_count}: Speed={speed:.1f} km/h, Steering={steering:.2f}")
            
            frame_count += 1
            time.sleep(0.05)  # 20 Hz

        print("Done!")

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        # Save episode data
        with open(f"{output_dir}/episode.json", "w") as f:
            json.dump(episode_data, f, indent=2)
        print(f"Saved episode data to {output_dir}/episode.json")
        
        # Cleanup
        for camera in cameras:
            camera.stop()
            camera.destroy()
        vehicle.destroy()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()
