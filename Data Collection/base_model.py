import carla
import time
import os
import shutil
import math
import json
import queue
import threading

image_queue = queue.Queue()

def cleanup_output_directory(output_dir):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    os.makedirs(f"{output_dir}/front")
    os.makedirs(f"{output_dir}/left")
    os.makedirs(f"{output_dir}/right")
    os.makedirs(f"{output_dir}/rear")
    print(f"Prepared: {output_dir}")

# ============================================
# RULE ENGINE (RSS-based)
# ============================================
class RuleEngine:
    def __init__(self):
        self.reaction_time = 0.5
        self.max_accel = 3.0
        self.min_brake = 4.0
        self.max_brake = 8.0
        
    def compute_safe_distance(self, ego_speed_ms, front_speed_ms=0):
        v_r = ego_speed_ms
        v_f = front_speed_ms
        rho = self.reaction_time
        d_min = (v_r * rho + 
                 (v_r ** 2) / (2 * self.min_brake) - 
                 (v_f ** 2) / (2 * self.max_brake))
        return max(d_min, 5.0)
    
    def evaluate(self, state):
        triggered_rules = []
        speed_ms = state['speed_kmh'] / 3.6
        
        action = {
            "action": "follow_lane",
            "speed": "moderate",
            "lane_position": "center"
        }
        
        if state['speed_kmh'] > state['speed_limit']:
            triggered_rules.append("speed_limit_exceeded")
            action["speed"] = "slow"
        
        if state['traffic_light'] == 'Red':
            if state['traffic_light_distance'] < 30:
                triggered_rules.append("red_light_stop")
                action = {"action": "stop", "urgency": "normal"}
            elif state['traffic_light_distance'] < 50:
                triggered_rules.append("red_light_approaching")
                action["speed"] = "slow"
        
        if state['traffic_light'] == 'Yellow':
            if state['traffic_light_distance'] < 20:
                triggered_rules.append("yellow_light_stop")
                action = {"action": "stop", "urgency": "normal"}
            else:
                triggered_rules.append("yellow_light_caution")
                action["speed"] = "slow"
        
        if state['vehicle_ahead_distance'] is not None:
            front_speed_ms = state['vehicle_ahead_speed'] / 3.6 if state['vehicle_ahead_speed'] else 0
            safe_dist = self.compute_safe_distance(speed_ms, front_speed_ms)
            
            if state['vehicle_ahead_distance'] < safe_dist * 0.5:
                triggered_rules.append("vehicle_too_close_emergency")
                action = {"action": "stop", "urgency": "emergency"}
            elif state['vehicle_ahead_distance'] < safe_dist:
                triggered_rules.append("vehicle_too_close")
                action = {"action": "slow_down", "intensity": "moderate"}
        
        if abs(state['lane_deviation']) > 0.3:
            triggered_rules.append("lane_correction_needed")
            if state['lane_deviation'] > 0:
                action["lane_position"] = "correct_left"
            else:
                action["lane_position"] = "correct_right"
        
        return action, triggered_rules
    
    def generate_reasoning(self, state, action, triggered_rules):
        parts = []
        parts.append(f"Speed: {state['speed_kmh']:.1f} km/h (limit: {state['speed_limit']} km/h).")
        
        if state['traffic_light']:
            parts.append(f"Traffic light: {state['traffic_light']} at {state['traffic_light_distance']:.1f}m.")
        
        if state['vehicle_ahead_distance']:
            parts.append(f"Vehicle ahead: {state['vehicle_ahead_distance']:.1f}m.")
        
        for rule in triggered_rules:
            if rule == "speed_limit_exceeded":
                parts.append("Exceeding speed limit, reducing speed.")
            elif rule == "red_light_stop":
                parts.append("Red light ahead, stopping.")
            elif rule == "yellow_light_stop":
                parts.append("Yellow light, stopping safely.")
            elif rule == "vehicle_too_close":
                parts.append("Vehicle ahead within RSS safe distance, slowing down.")
            elif rule == "vehicle_too_close_emergency":
                parts.append("Emergency: Vehicle too close, hard braking.")
            elif rule == "lane_correction_needed":
                parts.append("Drifting from lane center, correcting.")
        
        parts.append(f"Action: {action['action']}.")
        return " ".join(parts)

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

def get_lane_deviation(carla_map, vehicle):
    vehicle_location = vehicle.get_location()
    waypoint = carla_map.get_waypoint(vehicle_location)
    wp_loc = waypoint.transform.location
    dx = vehicle_location.x - wp_loc.x
    dy = vehicle_location.y - wp_loc.y
    yaw = math.radians(waypoint.transform.rotation.yaw)
    right_x = math.cos(yaw + math.pi/2)
    right_y = math.sin(yaw + math.pi/2)
    lateral_offset = dx * right_x + dy * right_y
    lane_width = waypoint.lane_width if waypoint.lane_width else 3.5
    deviation = lateral_offset / (lane_width / 2)
    return max(-1.0, min(1.0, deviation))

def get_traffic_light_state(vehicle, world):
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        state = traffic_light.get_state()
        tl_location = traffic_light.get_location()
        v_location = vehicle.get_location()
        distance = v_location.distance(tl_location)
        state_str = str(state).split('.')[-1]
        return state_str, distance
    return None, None

def get_vehicle_ahead(vehicle, world, max_distance=50):
    vehicle_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()
    vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
    forward_x = math.cos(vehicle_yaw)
    forward_y = math.sin(vehicle_yaw)
    closest_distance = None
    closest_speed = None
    
    for other in world.get_actors().filter('vehicle.*'):
        if other.id == vehicle.id:
            continue
        other_location = other.get_location()
        dx = other_location.x - vehicle_location.x
        dy = other_location.y - vehicle_location.y
        distance = math.sqrt(dx**2 + dy**2)
        if distance > max_distance:
            continue
        if distance > 0:
            dot = (dx * forward_x + dy * forward_y) / distance
            if dot > 0.8:
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
                    v = other.get_velocity()
                    closest_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6
    return closest_distance, closest_speed

def action_to_control(action, current_speed, target_speed_base=30):
    if action["action"] == "stop":
        target_speed = 0
    elif action["action"] == "slow_down":
        target_speed = target_speed_base * 0.5
    elif action.get("speed") == "slow":
        target_speed = target_speed_base * 0.7
    elif action.get("speed") == "fast":
        target_speed = target_speed_base * 1.2
    else:
        target_speed = target_speed_base
    
    if action["action"] == "stop":
        throttle = 0.0
        brake = 0.8 if action.get("urgency") == "emergency" else 0.5
    elif current_speed < target_speed - 2:
        throttle = 0.6
        brake = 0.0
    elif current_speed > target_speed + 2:
        throttle = 0.0
        brake = 0.3
    else:
        throttle = 0.3
        brake = 0.0
    
    return throttle, brake



# ============================================
# MAIN
# ============================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    
    print(f"Connected to: {carla_map.name}")
    
    # Cleanup
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('sensor.*'):
        actor.destroy()
    
    # Output directory
    episode_id = "ep_001"
    output_dir = f"data/{episode_id}"
    cleanup_output_directory(output_dir)
    
    # Spawn vehicle
    vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
    spawn_points = carla_map.get_spawn_points()
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print("Vehicle spawned.")
    
    vehicle_extent = vehicle.bounding_box.extent
    
    # ============================================
    # CAMERAS - Using YOUR working pattern
    # ============================================
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '105')
    camera_bp.set_attribute('sensor_tick', '0.05')  # 20 FPS (1/0.05 = 20)
    
    # Camera location (same for all, on roof)
    camera_location = carla.Location(
        x=vehicle_extent.x,
        y=0.0,
        z=vehicle_extent.z + 1.0
    )
    
    # Create transforms separately (YOUR working pattern)
    front_camera_rotation = carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)
    front_camera_transform = carla.Transform(camera_location, front_camera_rotation)
    
    left_camera_rotation = carla.Rotation(pitch=-10.0, yaw=-90.0, roll=0.0)
    left_camera_transform = carla.Transform(camera_location, left_camera_rotation)
    
    right_camera_rotation = carla.Rotation(pitch=-10.0, yaw=90.0, roll=0.0)
    right_camera_transform = carla.Transform(camera_location, right_camera_rotation)
    
    rear_camera_rotation = carla.Rotation(pitch=-10.0, yaw=180.0, roll=0.0)
    rear_camera_transform = carla.Transform(camera_location, rear_camera_rotation)
    
    # Spawn cameras
    front_cam = world.spawn_actor(camera_bp, front_camera_transform, attach_to=vehicle)
    left_cam = world.spawn_actor(camera_bp, left_camera_transform, attach_to=vehicle)
    right_cam = world.spawn_actor(camera_bp, right_camera_transform, attach_to=vehicle)
    rear_cam = world.spawn_actor(camera_bp, rear_camera_transform, attach_to=vehicle)
    
    # Listen with YOUR working pattern
    camera_dict = {
        "front": front_cam,
        "left": left_cam,
        "right": right_cam,
        "rear": rear_cam
    }
    
    # Start asynchronous image saving thread
    def image_saving_thread():
        while True:
            try:
                filepath, image_data = image_queue.get(timeout=1)
                image_data.save_to_disk(filepath)
            except queue.Empty:
                continue
    
    threading.Thread(target=image_saving_thread, daemon=True).start()

    def save_image_async(image, angle):
        filepath = f'{output_dir}/{angle}/{image.frame}.png'
        image_queue.put((filepath, image))



    for angle, cam in camera_dict.items():
        cam.listen(lambda image, angle=angle: save_image_async(image, angle))
    
    cameras = list(camera_dict.values())
    print("4 cameras attached.")


    # Initialize rule engine
    rule_engine = RuleEngine()
    
    # Data storage
    episode_data = []
    
    # Driving parameters
    target_speed_base = 30
    frame_count = 0
    max_frames = 600  # 30 seconds
    
    try:
        print("Starting to drive with rule engine...")
        
        while frame_count < max_frames:
            world.wait_for_tick()
            speed_kmh = get_speed_kmh(vehicle)
            lane_deviation = get_lane_deviation(carla_map, vehicle)
            traffic_light, tl_distance = get_traffic_light_state(vehicle, world)
            vehicle_ahead_dist, vehicle_ahead_speed = get_vehicle_ahead(vehicle, world)
            current_wp = carla_map.get_waypoint(vehicle.get_location())
            speed_limit = 30
            
            state = {
                'speed_kmh': speed_kmh,
                'speed_limit': speed_limit,
                'traffic_light': traffic_light,
                'traffic_light_distance': tl_distance if tl_distance else 100,
                'vehicle_ahead_distance': vehicle_ahead_dist,
                'vehicle_ahead_speed': vehicle_ahead_speed,
                'at_intersection': current_wp.is_junction,
                'lane_deviation': lane_deviation,
            }
            
            action, triggered_rules = rule_engine.evaluate(state)
            reasoning = rule_engine.generate_reasoning(state, action, triggered_rules)
            
            next_wp = get_next_waypoint(carla_map, vehicle, distance=5.0)
            steering = get_steering(vehicle, next_wp.transform.location)
            throttle, brake = action_to_control(action, speed_kmh, target_speed_base)
            
            control = carla.VehicleControl()
            control.steer = steering
            control.throttle = throttle
            control.brake = brake
            vehicle.apply_control(control)
            
            # Record data every 20 frames (~1 sec)
            if frame_count % 20 == 0:
                location = vehicle.get_location()
                rotation = vehicle.get_transform().rotation
                
                frame_data = {
                    "frame": frame_count,
                    "timestamp": time.time(),
                    "state": {
                        "speed_kmh": round(speed_kmh, 2),
                        "speed_limit": speed_limit,
                        "location": {"x": round(location.x, 2), "y": round(location.y, 2), "z": round(location.z, 2)},
                        "rotation": {"pitch": round(rotation.pitch, 2), "yaw": round(rotation.yaw, 2), "roll": round(rotation.roll, 2)},
                        "lane_deviation": round(lane_deviation, 3),
                        "traffic_light": traffic_light,
                        "traffic_light_distance": round(tl_distance, 2) if tl_distance else None,
                        "vehicle_ahead_distance": round(vehicle_ahead_dist, 2) if vehicle_ahead_dist else None,
                        "vehicle_ahead_speed": round(vehicle_ahead_speed, 2) if vehicle_ahead_speed else None,
                        "at_intersection": current_wp.is_junction,
                    },
                    "control": {
                        "steering": round(steering, 3),
                        "throttle": round(throttle, 2),
                        "brake": round(brake, 2),
                    },
                    "action": action,
                    "triggered_rules": triggered_rules,
                    "reasoning": reasoning,
                }
                episode_data.append(frame_data)
                print(f"Frame {frame_count}: {speed_kmh:.1f}km/h | Rules: {triggered_rules} | Action: {action['action']}")
            
            frame_count += 1
            
        
        print("Episode complete!")
    
    except KeyboardInterrupt:
        print("Stopped by user.")
    
    finally:
        with open(f"{output_dir}/episode.json", "w") as f:
            json.dump(episode_data, f, indent=2)
        print(f"Saved {len(episode_data)} frames to {output_dir}/episode.json")
        
        for cam in cameras:
            cam.stop()
            cam.destroy()
        vehicle.destroy()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()