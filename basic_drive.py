import carla
import math
import time

# ============================================
# CONNECT
# ============================================
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
carla_map = world.get_map()

print(f"Connected to: {carla_map.name}")

# ============================================
# CLEANUP OLD ACTORS
# ============================================
for actor in world.get_actors().filter('vehicle.*'):
    actor.destroy()
print("Cleaned up old vehicles")

time.sleep(1)  # Wait a moment for cleanup

# ============================================
# SPAWN VEHICLE
# ============================================
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

spawn_points = carla_map.get_spawn_points()
start_point = spawn_points[0]

vehicle = world.spawn_actor(vehicle_bp, start_point)
print(f"Vehicle spawned at: {start_point.location}")

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

# ============================================
# MAIN LOOP
# ============================================
target_speed = 30
frame_count = 0
max_frames = 2000

try:
    print("Starting to drive...")
    
    while frame_count < max_frames:
        next_wp = get_next_waypoint(carla_map, vehicle, distance=5.0)
        target_loc = next_wp.transform.location
        
        steering = get_steering(vehicle, target_loc)
        speed = get_speed_kmh(vehicle)
        
        if speed < target_speed:
            throttle = 0.6
            brake = 0.0
        else:
            throttle = 0.0
            brake = 0.3
        
        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        vehicle.apply_control(control)
        
        if frame_count % 100 == 0:
            print(f"Frame {frame_count}: Speed={speed:.1f} km/h, Steering={steering:.2f}")
        
        frame_count += 1
        time.sleep(0.05)

    print("Done!")

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    vehicle.destroy()
    print("Vehicle destroyed")