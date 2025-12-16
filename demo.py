import carla
import time
import os
import queue
import threading
import numpy as np
from PIL import Image

# ============================================
# CONFIG
# ============================================
OUTPUT_DIR = "test_queue"
DURATION = 30  # seconds

# ============================================
# QUEUE + THREADING
# ============================================
image_queue = queue.Queue()
frame_count = {"front": 0, "left": 0, "right": 0, "rear": 0}

def camera_callback(image, angle):
    """Just queue raw bytes - don't save here"""
    image_queue.put({
        "frame": image.frame,
        "angle": angle,
        "data": bytes(image.raw_data),
        "height": image.height,
        "width": image.width
    })
    frame_count[angle] += 1

def image_saver():
    """Background thread saves images"""
    while True:
        try:
            item = image_queue.get(timeout=10)
            if item is None:
                break
            
            array = np.frombuffer(item["data"], dtype=np.uint8)
            array = array.reshape((item["height"], item["width"], 4))[:, :, :3]
            
            img = Image.fromarray(array)
            img.save(f'{OUTPUT_DIR}/{item["angle"]}/{item["frame"]:06d}.png')
            
        except queue.Empty:
            continue

# ============================================
# MAIN
# ============================================
def main():
    # Connect
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    
    print(f"Connected: {world.get_map().name}")
    
    # Cleanup
    for a in world.get_actors().filter('vehicle.*'):
        a.destroy()
    for a in world.get_actors().filter('sensor.*'):
        a.destroy()
    
    # Output dirs
    os.makedirs(f"{OUTPUT_DIR}/front", exist_ok=True)
    os.makedirs(f"{OUTPUT_DIR}/left", exist_ok=True)
    os.makedirs(f"{OUTPUT_DIR}/right", exist_ok=True)
    os.makedirs(f"{OUTPUT_DIR}/rear", exist_ok=True)
    
    # Start saver thread
    saver_thread = threading.Thread(target=image_saver, daemon=True)
    saver_thread.start()
    
    # Spawn vehicle
    vehicle_bp = bp_lib.find("vehicle.tesla.model3")
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)
    print("Vehicle spawned")
    
    extent = vehicle.bounding_box.extent
    
    # Cameras
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1920')
    cam_bp.set_attribute('image_size_y', '1080')
    cam_bp.set_attribute('fov', '105')
    cam_bp.set_attribute('sensor_tick', '0.5')  # 2 FPS
    
    cam_loc = carla.Location(x=extent.x, y=0.0, z=extent.z + 1.0)
    
    cams = {}
    rots = {
        "front": carla.Rotation(pitch=-10, yaw=0, roll=0),
        "left": carla.Rotation(pitch=-10, yaw=-90, roll=0),
        "right": carla.Rotation(pitch=-10, yaw=90, roll=0),
        "rear": carla.Rotation(pitch=-10, yaw=180, roll=0),
    }
    
    for name, rot in rots.items():
        t = carla.Transform(cam_loc, rot)
        c = world.spawn_actor(cam_bp, t, attach_to=vehicle)
        c.listen(lambda img, n=name: camera_callback(img, n))
        cams[name] = c
    
    print("Cameras attached")
    print(f"Recording for {DURATION} seconds...")
    
    # Run
    start = time.time()
    while time.time() - start < DURATION:
        time.sleep(1)
        elapsed = int(time.time() - start)
        print(f"  {elapsed}s | Queue: {image_queue.qsize()} | Saved: {frame_count}")
    
    # Stop cameras
    print("Stopping cameras...")
    for c in cams.values():
        c.stop()
    
    # Wait for queue to empty
    print(f"Flushing queue ({image_queue.qsize()} remaining)...")
    while not image_queue.empty():
        time.sleep(0.5)
        print(f"  Queue: {image_queue.qsize()}")
    
    time.sleep(3)
    
    # Cleanup
    for c in cams.values():
        c.destroy()
    vehicle.destroy()
    
    # Stats
    print("\n" + "="*50)
    print("RESULTS")
    print("="*50)
    for angle in ['front', 'left', 'right', 'rear']:
        count = len(os.listdir(f"{OUTPUT_DIR}/{angle}"))
        print(f"{angle}: {count} images")
    
    total = sum(len(os.listdir(f"{OUTPUT_DIR}/{a}")) for a in ['front', 'left', 'right', 'rear'])
    print(f"\nTotal: {total} images")
    print(f"Expected: {DURATION * 2 * 4} images (4 cams × 2 FPS × {DURATION}s)")

if __name__ == "__main__":
    main()