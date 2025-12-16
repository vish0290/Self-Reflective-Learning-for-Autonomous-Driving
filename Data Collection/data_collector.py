
import carla
import time
import os
import shutil
import json
import math

# ============================================
# CONFIG
# ============================================
OUTPUT_BASE = "./data"
NUM_EPISODES = 20
EPISODE_DURATION = 60  # seconds
SPEED_LIMIT = 30  # km/h

# ============================================
# RULE ENGINE (5 Rules)
# ============================================
class RuleEngine:
    def __init__(self):
        self.reaction_time = 0.5
        self.min_brake = 4.0
        self.max_brake = 8.0

    def compute_safe_distance(self, speed_ms):
        return speed_ms * self.reaction_time + (speed_ms ** 2) / (2 * self.min_brake)

    def classify_steering(self, steering_angle):
        """Classify steering angle into text description"""
        if abs(steering_angle) < 0.05:
            return "straight"
        elif abs(steering_angle) < 0.15:
            return "slight_left" if steering_angle < 0 else "slight_right"
        elif abs(steering_angle) < 0.35:
            return "moderate_left" if steering_angle < 0 else "moderate_right"
        else:
            return "sharp_left" if steering_angle < 0 else "sharp_right"

    def classify_speed(self, speed_action, is_intersection):
        """Classify speed action into text description"""
        if speed_action == "stop":
            return "stop"
        elif speed_action == "slow_down":
            return "decelerate"
        elif speed_action == "slow":
            return "reduce_speed"
        elif is_intersection:
            return "slow_for_intersection"
        else:
            return "maintain_speed"

    def classify_lane_position(self, lane_position, lane_deviation):
        """Classify lane position"""
        if lane_position == "correct_left":
            return "correcting_to_center_from_right"
        elif lane_position == "correct_right":
            return "correcting_to_center_from_left"
        elif abs(lane_deviation) < 0.1:
            return "center_lane"
        elif lane_deviation > 0:
            return "right_of_center"
        else:
            return "left_of_center"

    def evaluate(self, speed_kmh, speed_limit, vehicle_ahead_dist, traffic_light, tl_distance, lane_deviation, is_intersection, steering_angle):
        triggered = []
        action = {"action": "follow_lane", "speed": "normal", "lane_position": "center"}

        speed_ms = speed_kmh / 3.6
        safe_dist = max(self.compute_safe_distance(speed_ms), 5.0)

        # Rule 1: Speed limit (Vienna Conv. Art. 13)
        if speed_kmh > speed_limit:
            triggered.append({"rule": "speed_limit", "source": "Vienna Convention Art. 13"})
            action["speed"] = "slow"

        # Rule 2: Vehicle ahead - RSS (Shalev-Shwartz 2017)
        if vehicle_ahead_dist is not None:
            if vehicle_ahead_dist < safe_dist * 0.5:
                triggered.append({"rule": "emergency_stop", "source": "RSS Lemma 2", "safe_dist": round(safe_dist, 1)})
                action = {"action": "stop", "urgency": "emergency"}
            elif vehicle_ahead_dist < safe_dist:
                triggered.append({"rule": "too_close", "source": "RSS Lemma 2", "safe_dist": round(safe_dist, 1)})
                action = {"action": "slow_down"}

        # Rule 3: Red light (Vienna Conv. Art. 23)
        if traffic_light == "Red" and tl_distance and tl_distance < 30:
            triggered.append({"rule": "red_light", "source": "Vienna Convention Art. 23"})
            action = {"action": "stop"}

        # Rule 4: Yellow light (Vienna Conv. Art. 23)
        if traffic_light == "Yellow" and tl_distance and tl_distance < 20:
            triggered.append({"rule": "yellow_light", "source": "Vienna Convention Art. 23"})
            action = {"action": "stop"}

        # Rule 5: Lane keeping (Vienna Conv. Art. 11)
        if abs(lane_deviation) > 0.3:
            triggered.append({"rule": "lane_correction", "source": "Vienna Convention Art. 11"})
            if lane_deviation > 0:
                action["lane_position"] = "correct_left"
            else:
                action["lane_position"] = "correct_right"

        # Rule 6: Intersection (Vienna Conv. Art. 18)
        if is_intersection and action["action"] not in ["stop", "slow_down"]:
            triggered.append({"rule": "intersection_approach", "source": "Vienna Convention Art. 18"})
            if "speed" not in action or action["speed"] == "normal":
                action["speed"] = "slow"

        # Create formatted action string
        lane_pos = self.classify_lane_position(action.get("lane_position", "center"), lane_deviation)
        speed_class = self.classify_speed(action.get("action", "follow_lane"), is_intersection)
        steer_class = self.classify_steering(steering_angle)

        action["formatted"] = f"{lane_pos}|{speed_class}|{steer_class}"
        action["is_intersection"] = is_intersection

        return action, triggered

# ============================================
# HELPERS
# ============================================
def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

def get_next_waypoint(world_map, vehicle, distance=5.0):
    loc = vehicle.get_location()
    wp = world_map.get_waypoint(loc)
    next_wps = wp.next(distance)
    return next_wps[0] if next_wps else wp

def get_steering(vehicle, target_loc):
    transform = vehicle.get_transform()
    loc = transform.location
    yaw = transform.rotation.yaw
    
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    
    target_yaw = math.degrees(math.atan2(dy, dx))
    error = target_yaw - yaw
    
    while error > 180: error -= 360
    while error < -180: error += 360
    
    return max(-1.0, min(1.0, error / 30.0))

def get_lane_deviation(world_map, vehicle):
    loc = vehicle.get_location()
    wp = world_map.get_waypoint(loc)
    wp_loc = wp.transform.location
    
    dx = loc.x - wp_loc.x
    dy = loc.y - wp_loc.y
    
    yaw = math.radians(wp.transform.rotation.yaw)
    right_x = math.cos(yaw + math.pi/2)
    right_y = math.sin(yaw + math.pi/2)
    
    lateral = dx * right_x + dy * right_y
    lane_width = wp.lane_width if wp.lane_width else 3.5
    
    return max(-1.0, min(1.0, lateral / (lane_width / 2)))

def get_traffic_light(vehicle):
    if vehicle.is_at_traffic_light():
        tl = vehicle.get_traffic_light()
        state = str(tl.get_state()).split('.')[-1]
        dist = vehicle.get_location().distance(tl.get_location())
        return state, dist
    return None, None

def get_vehicle_ahead(vehicle, world):
    loc = vehicle.get_location()
    fwd = vehicle.get_transform().get_forward_vector()

    for other in world.get_actors().filter('vehicle.*'):
        if other.id == vehicle.id:
            continue
        other_loc = other.get_location()
        diff_x = other_loc.x - loc.x
        diff_y = other_loc.y - loc.y
        dist = math.sqrt(diff_x**2 + diff_y**2)

        if dist > 50:
            continue

        dot = (diff_x * fwd.x + diff_y * fwd.y) / dist if dist > 0 else 0
        if dot > 0.8:
            return dist
    return None

def is_at_intersection(world_map, vehicle):
    """Check if vehicle is approaching or at an intersection"""
    loc = vehicle.get_location()
    wp = world_map.get_waypoint(loc)

    # Check if current waypoint is a junction
    if wp.is_junction:
        return True

    # Check if intersection is ahead (within 15 meters)
    next_wps = wp.next(15.0)
    for next_wp in next_wps:
        if next_wp.is_junction:
            return True

    return False

def action_to_control(action, speed_kmh, target_speed=30):
    if action["action"] == "stop":
        throttle = 0.0
        brake = 0.8 if action.get("urgency") == "emergency" else 0.5
    elif action["action"] == "slow_down":
        target = target_speed * 0.5
        throttle = 0.4 if speed_kmh < target else 0.0
        brake = 0.3 if speed_kmh > target else 0.0
    elif action.get("speed") == "slow":
        target = target_speed * 0.7
        throttle = 0.5 if speed_kmh < target else 0.0
        brake = 0.2 if speed_kmh > target else 0.0
    else:
        throttle = 0.6 if speed_kmh < target_speed else 0.0
        brake = 0.2 if speed_kmh > target_speed + 5 else 0.0
    
    return throttle, brake

def cleanup_dir(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.makedirs(path)
    os.makedirs(f"{path}/front")
    os.makedirs(f"{path}/left")
    os.makedirs(f"{path}/right")
    os.makedirs(f"{path}/rear")

# ============================================
# COLLECT EPISODE
# ============================================
def collect_episode(client, episode_id):
    world = client.get_world()
    world_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    
    # Cleanup
    for a in world.get_actors().filter('vehicle.*'):
        a.destroy()
    for a in world.get_actors().filter('sensor.*'):
        a.destroy()
    time.sleep(1)
    
    # Output
    output_dir = f"{OUTPUT_BASE}/ep_{episode_id:03d}"
    cleanup_dir(output_dir)
    
    # Spawn vehicle at different points
    spawn_points = world_map.get_spawn_points()
    spawn_idx = episode_id % len(spawn_points)
    vehicle_bp = bp_lib.find("vehicle.tesla.model3")
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[spawn_idx])
    print(f"  Spawned at point {spawn_idx}")
    
    extent = vehicle.bounding_box.extent
    
    # Cameras
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '960')
    cam_bp.set_attribute('image_size_y', '540')
    cam_bp.set_attribute('fov', '105')
    cam_bp.set_attribute('sensor_tick', '0.5')
    
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
        c.listen(lambda img, n=name: img.save_to_disk(f'{output_dir}/{n}/{img.frame:06d}.png'))
        cams[name] = c
    
    print("  Cameras ready")
    
    # Rule engine
    rule_engine = RuleEngine()
    
    # Collect
    frames = []
    start = time.time()
    
    try:
        while time.time() - start < EPISODE_DURATION:
            # State
            speed_kmh = get_speed_kmh(vehicle)
            tl_state, tl_dist = get_traffic_light(vehicle)
            veh_ahead = get_vehicle_ahead(vehicle, world)
            lane_dev = get_lane_deviation(world_map, vehicle)
            at_intersection = is_at_intersection(world_map, vehicle)

            # Steering (calculate before rules so we can pass it)
            next_wp = get_next_waypoint(world_map, vehicle)
            steering = get_steering(vehicle, next_wp.transform.location)

            # Rules
            action, triggered = rule_engine.evaluate(
                speed_kmh=speed_kmh,
                speed_limit=SPEED_LIMIT,
                vehicle_ahead_dist=veh_ahead,
                traffic_light=tl_state,
                tl_distance=tl_dist,
                lane_deviation=lane_dev,
                is_intersection=at_intersection,
                steering_angle=steering
            )
            
            # Control
            throttle, brake = action_to_control(action, speed_kmh, SPEED_LIMIT)
            
            ctrl = carla.VehicleControl()
            ctrl.steer = steering
            ctrl.throttle = throttle
            ctrl.brake = brake
            vehicle.apply_control(ctrl)
            
            # Record
            snapshot = world.get_snapshot()
            loc = vehicle.get_location()
            rot = vehicle.get_transform().rotation
            
            frames.append({
                "frame": snapshot.frame,
                "time": round(time.time() - start, 2),
                "speed_kmh": round(speed_kmh, 2),
                "location": {"x": round(loc.x, 2), "y": round(loc.y, 2), "z": round(loc.z, 2)},
                "rotation": {"yaw": round(rot.yaw, 2)},
                "control": {"steer": round(steering, 3), "throttle": round(throttle, 2), "brake": round(brake, 2)},
                "action": action,
                "triggered_rules": triggered,
                "traffic_light": tl_state,
                "traffic_light_dist": round(tl_dist, 1) if tl_dist else None,
                "vehicle_ahead_m": round(veh_ahead, 1) if veh_ahead else None,
                "lane_deviation": round(lane_dev, 3)
            })
            
            elapsed = int(time.time() - start)
            if elapsed % 15 == 0 and len(frames) % 30 == 1:
                rules_str = [r["rule"] for r in triggered] if triggered else ["none"]
                print(f"    {elapsed}s | {speed_kmh:.1f}km/h | Rules: {rules_str}")
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("  Stopped")

    # Stop cameras immediately to prevent extra frames
    for c in cams.values():
        c.stop()

    # Flush
    print("  Saving...")
    time.sleep(5)
    
    for c in cams.values():
        c.destroy()
    vehicle.destroy()
    
    # Save JSON
    with open(f"{output_dir}/episode.json", "w") as f:
        json.dump({"episode_id": episode_id, "frames": frames}, f, indent=2)
    
    img_count = len(os.listdir(f"{output_dir}/front"))
    print(f"  Done: {img_count} images, {len(frames)} frames\n")
    return img_count

# ============================================
# MAIN
# ============================================
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    print(f"Connected: {client.get_world().get_map().name}\n")
    
    os.makedirs(OUTPUT_BASE, exist_ok=True)
    
    total_images = 0
    for ep in range(NUM_EPISODES):
        print(f"[Episode {ep+1}/{NUM_EPISODES}]")
        try:
            count = collect_episode(client, ep)
            total_images += count
        except Exception as e:
            print(f"  Failed: {e}\n")
        time.sleep(3)
    
    print("=" * 50)
    print(f"COMPLETE: {NUM_EPISODES} episodes, ~{total_images} images")
    print(f"Data saved to: {OUTPUT_BASE}")
    print("=" * 50)

if __name__ == "__main__":
    main()
