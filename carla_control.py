import carla
import time
import random
import os
import shutil
import numpy as np
import base64
import cv2
from PIL import Image
from io import BytesIO
import base64
from db import push_frame, stream_setup, health_check


def cleanup_output_directory(output_dir):
    # Delete any existing files in the output directory
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    print(f"Cleaned up and prepared output directory: {output_dir}")

class EgoState:
    def __init__(self):
        self.img_base64 = None # Base64 encoded JPEG
def cam_feed(image, ego_state: EgoState):
    try:
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        _, buffer = cv2.imencode('.jpg', array)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        ego_state.img_base64 = jpg_as_text
        push_frame(jpg_as_text)

    except Exception as e:
        with open("error_log.txt", "a") as f:
            f.write(f"Error processing camera feed: {e}\n")
        print(f"Error processing camera feed: {e}")

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
    camera_bp.set_attribute('image_size_x', '960')
    camera_bp.set_attribute('image_size_y', '540')
    camera_bp.set_attribute('fov', '105') # Field of view in degrees
    camera_bp.set_attribute('sensor_tick', '1.0') # Capture image every second

    # Calculate camera location based on vehicle dimensions
    camera_location = carla.Location(
        x=vehicle_extent.x ,    # Slightly forward from the vehicle's front
        y=0.0,                       # Centered horizontally on the vehicle
        z=vehicle_extent.z + 1.0     # Elevated above the roof
    )

    # Adjust camera rotation to face forward
    camera_rotation = carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)  # Tilt slightly downward
    camera_transform = carla.Transform(camera_location, camera_rotation)
    
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    ego_state = EgoState()
    camera.listen(lambda image: cam_feed(image, ego_state))
    # camera.listen(lambda image: cam_feed(image))
    
    
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
            if time.time() - start_time > 5:
                if ego_state.img_base64:
                    print(f"Captured image (base64 size): {len(ego_state.img_base64)} characters")
                else:
                    print("No image captured yet.")
    
    finally:
        # Clean up by destroying actors
        if camera:
            camera.stop()
            camera.destroy()
        vehicle.destroy()
        print("Actors destroyed, simulation finished.")

if __name__ == "__main__":
    stream_setup()
    main()