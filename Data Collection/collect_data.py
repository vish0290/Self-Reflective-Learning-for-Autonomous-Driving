import carla
import time
import os
import shutil
import json

def cleanup_output_directory(output_dir):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    os.makedirs(f"{output_dir}/front")
    os.makedirs(f"{output_dir}/left")
    os.makedirs(f"{output_dir}/right")
    os.makedirs(f"{output_dir}/rear")
    print(f"Prepared: {output_dir}")

def collect_episode(client, episode_id, duration=60):
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    
    # Cleanup
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('sensor.*'):
        actor.destroy()
    
    # Output directory
    output_dir = f"/runpod-volume/data/ep_{episode_id:03d}"
    cleanup_output_directory(output_dir)
    
    # Spawn vehicle
    vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[episode_id % len(spawn_points)]  # Different start each episode
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)
    print(f"Episode {episode_id}: Vehicle spawned at spawn point {episode_id % len(spawn_points)}")
    
    vehicle_extent = vehicle.bounding_box.extent
    
    # Camera setup
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '105')
    camera_bp.set_attribute('sensor_tick', '0.5')  # 2 FPS
    
    camera_location = carla.Location(
        x=vehicle_extent.x,
        y=0.0,
        z=vehicle_extent.z + 1.0
    )
    
    # Spawn 4 cameras
    cameras = {}
    rotations = {
        "front": carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0),
        "left": carla.Rotation(pitch=-10.0, yaw=-90.0, roll=0.0),
        "right": carla.Rotation(pitch=-10.0, yaw=90.0, roll=0.0),
        "rear": carla.Rotation(pitch=-10.0, yaw=180.0, roll=0.0),
    }
    
    for angle, rotation in rotations.items():
        transform = carla.Transform(camera_location, rotation)
        cam = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        cam.listen(lambda img, a=angle: img.save_to_disk(f'{output_dir}/{a}/{img.frame:06d}.png'))
        cameras[angle] = cam
    
    print("4 cameras attached")
    
    # Data collection
    frame_data = []
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            world.wait_for_tick()
            
            # Get vehicle state
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            
            location = vehicle.get_location()
            rotation = vehicle.get_transform().rotation
            control = vehicle.get_control()
            
            # Get current frame
            snapshot = world.get_snapshot()
            frame = snapshot.frame
            
            # Record data
            data = {
                "frame": frame,
                "timestamp": time.time() - start_time,
                "speed_kmh": round(speed_kmh, 2),
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
                "control": {
                    "throttle": round(control.throttle, 3),
                    "steer": round(control.steer, 3),
                    "brake": round(control.brake, 3)
                }
            }
            frame_data.append(data)
            
            # Print progress
            elapsed = int(time.time() - start_time)
            if elapsed % 10 == 0 and len(frame_data) % 20 == 0:
                print(f"  {elapsed}s: {speed_kmh:.1f} km/h | Frames: {len(frame_data)}")
    
    except KeyboardInterrupt:
        print("Stopped early")
    
    # Buffer before cleanup
    print("Flushing buffers...")
    time.sleep(5)
    
    # Stop cameras
    for cam in cameras.values():
        cam.stop()
    time.sleep(2)
    
    # Destroy actors
    for cam in cameras.values():
        cam.destroy()
    vehicle.destroy()
    
    # Save metadata
    with open(f"{output_dir}/episode.json", "w") as f:
        json.dump({
            "episode_id": episode_id,
            "duration": duration,
            "total_frames": len(frame_data),
            "frames": frame_data
        }, f, indent=2)
    
    # Count images
    front_count = len(os.listdir(f"{output_dir}/front"))
    print(f"Episode {episode_id} complete: {front_count} images, {len(frame_data)} data points")
    
    return front_count

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    
    print(f"Connected to: {client.get_world().get_map().name}")
    
    NUM_EPISODES = 30
    EPISODE_DURATION = 60  # seconds
    
    for ep in range(NUM_EPISODES):
        print(f"\n{'='*50}")
        print(f"EPISODE {ep + 1}/{NUM_EPISODES}")
        print(f"{'='*50}")
        
        try:
            collect_episode(client, ep, EPISODE_DURATION)
        except Exception as e:
            print(f"Episode {ep} failed: {e}")
            continue
        
        # Short break between episodes
        time.sleep(3)
    
    print("\n" + "="*50)
    print("DATA COLLECTION COMPLETE")
    print("="*50)

if __name__ == "__main__":
    main()
