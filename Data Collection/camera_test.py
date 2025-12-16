import carla
import time
import random
import os
import shutil

def cleanup_output_directory(output_dir):
    # Delete any existing files in the output directory
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    print(f"Cleaned up and prepared output directory: {output_dir}")

def main():
    ############## SETTINGS #################

    # Connect to CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Load the world and retrieve the blueprint library
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('sensor.*'):
        actor.destroy()

    # Spectator - Pointing to spawn point 0
    # spectator = world.get_spectator()
    # camera_loc = carla.Location(x=110.02999877929688, y=216.0, z=50.0)
    # camera_rot = carla.Rotation(pitch=-69.0, yaw=0.0, roll=0.0)
    # camera_trans = carla.Transform(camera_loc, camera_rot)
    
    # # Move the spectator
    # spectator.set_transform(camera_trans)

    ############## CAR #################

    # Find a cybertruck blueprint
    vehicle_bp = blueprint_library.find("vehicle.tesla.model3") 
    
    # Choose a spawn point for the car
    spawn_points = world.get_map().get_spawn_points()
    vehicle_spawn_point = spawn_points[0]

    # Spawn the vehicle
    vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
    print("Vehicle spawned.")

    # Retrieve vehicle dimensions
    vehicle_extent = vehicle.bounding_box.extent
    vehicle_length = vehicle_extent.x * 2  # Full length of the vehicle
    vehicle_width = vehicle_extent.y * 2   # Full width of the vehicle
    vehicle_height = vehicle_extent.z * 2  # Full height of the vehicle

    print(f"Vehicle dimensions - Length: {vehicle_length} m, Width: {vehicle_width} m, Height: {vehicle_height} m")

    # Enable autopilot
    vehicle.set_autopilot(True)
    print("Autopilot enabled.")

    ############## CAMERA #################

    # Find a camera blueprint
    camera_bp = blueprint_library.find('sensor.camera.rgb')

    # Adjust camera attributes (optional)
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '105') # Field of view in degrees
    camera_bp.set_attribute('sensor_tick', '1.0') # Capture image every second

    # Calculate camera location based on vehicle dimensions
    camera_location = carla.Location(
        x=vehicle_extent.x ,    # Slightly forward from the vehicle's front
        y=0.0,                       # Centered horizontally on the vehicle
        z=vehicle_extent.z + 1.0     # Elevated above the roof
    )

    # Adjust camera rotation to face forward
    front_camera_rotation = carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)  # Tilt slightly downward
    front_camera_transform = carla.Transform(camera_location, front_camera_rotation)
    left_camera_rotation = carla.Rotation(pitch=-10.0, yaw=90.0, roll=0.0)  # Tilt slightly downward
    left_camera_transform = carla.Transform(camera_location, left_camera_rotation)
    right_camera_rotation = carla.Rotation(pitch=-10.0, yaw=-90.0, roll=0.0)  # Tilt slightly downward
    right_camera_transform = carla.Transform(camera_location, right_camera_rotation)
    rear_camera_rotation = carla.Rotation(pitch=-10.0, yaw=180.0, roll=0.0)  # Tilt slightly downward
    rear_camera_transform = carla.Transform(camera_location, rear_camera_rotation)
    
    front_camera = world.spawn_actor(camera_bp, front_camera_transform, attach_to=vehicle)
    left_camera = world.spawn_actor(camera_bp, left_camera_transform, attach_to=vehicle)
    right_camera = world.spawn_actor(camera_bp, right_camera_transform, attach_to=vehicle)
    rear_camera = world.spawn_actor(camera_bp, rear_camera_transform, attach_to=vehicle)
    print("Cameras spawned and attached to vehicle.")
    output_dir = "images"
    if os.path.exists("images") == False:
        os.makedirs("images")
    cleanup_output_directory(output_dir)
    camera_angles = {"front": front_camera, "left": left_camera, "right": right_camera, "rear": rear_camera}
    for angle, cam in camera_angles.items():
        print(f"{angle.capitalize()} camera spawned.")
        if angle not in os.listdir("images"):
            os.makedirs(f"images/{angle}")
        cam.listen(lambda image, angle=angle: image.save_to_disk(f'images/{angle}/{image.frame}.png'))
    
    
# Velocity & Speed
    velocity = vehicle.get_velocity()  # m/s vector
    speed_ms = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
    speed_kmh = 3.6 * speed_ms

    # Position & Rotation
    transform = vehicle.get_transform()
    location = transform.location  # x, y, z
    rotation = transform.rotation  # pitch, yaw, roll

    # Acceleration
    acceleration = vehicle.get_acceleration()

    # Angular velocity
    angular_velocity = vehicle.get_angular_velocity()

    # Control inputs (steering, throttle, brake)
    control = vehicle.get_control()
    print(f"Throttle: {control.throttle}, Steering: {control.steer}, Brake: {control.brake}")

    ############## RUN SIMULATION #################

    try:
        # Let the simulation run for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            world.wait_for_tick()
            
            # Get vehicle metadata
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            
            location = vehicle.get_location()
            transform = vehicle.get_transform()
            
            print(f"Speed: {speed_kmh:.2f} km/h | Location: ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})")
    
    finally:
        # Clean up by destroying actors
        for camera in camera_angles.values():
            camera.stop()
            camera.destroy()
        vehicle.destroy()
        print("Actors destroyed, simulation finished.")

if __name__ == "__main__":
    main()