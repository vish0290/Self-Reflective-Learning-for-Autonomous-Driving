"""
================================================================================
CARLA + VLM Trajectory Prediction with Pygame Visualization
================================================================================

Master Thesis: Self-Reflective Learning for Autonomous Driving
Author: Vishwanath Ajith Sardeshpande

Features:
- CARLA autopilot data collection with ground truth trajectories
- VLM pixel-based trajectory prediction
- Side-by-side visualization (Ground Truth vs VLM Prediction)
- Real-time pygame display with vehicle state
- Optional: VLM-controlled driving mode

Controls:
- SPACE: Toggle between Autopilot and VLM control
- V: Toggle VLM prediction ON/OFF
- S: Save current frame
- R: Reset vehicle
- ESC: Quit

================================================================================
"""

import carla
import pygame
import numpy as np
import math
import time
import json
import os
import threading
import queue
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
from PIL import Image
from io import BytesIO
import base64
import re

# For VLM
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    # CARLA
    CARLA_HOST: str = "localhost"
    CARLA_PORT: int = 2000
    
    # Camera (must match VLM training)
    CAMERA_WIDTH: int = 1920
    CAMERA_HEIGHT: int = 1080
    CAMERA_FOV: int = 115
    
    # Display
    DISPLAY_WIDTH: int = 1920
    DISPLAY_HEIGHT: int = 1080
    
    # VLM
    VLM_MODEL: str = "Qwen/Qwen3-VL-8B-Instruct"
    VLM_BASE_URL: str = "https://vish2kber--vlm-inference-serve.modal.run/v1"
    VLM_TEMPERATURE: float = 0.2
    VLM_INTERVAL: float = 1.0  # seconds between predictions
    
    # Trajectory
    NUM_WAYPOINTS: int = 10
    WAYPOINT_DEPTHS: Tuple[float, ...] = (2, 4, 6, 8, 10, 13, 16, 20, 25, 30)
    
    # Control
    TARGET_SPEED: float = 30.0  # km/h


CONFIG = Config()


# =============================================================================
# VLM SYSTEM PROMPT
# =============================================================================

SYSTEM_PROMPT = """You are an autonomous vehicle trajectory predictor. Predict the driving path as 10 pixel coordinates on the camera image.

IMAGE: 1920 x 1080 pixels
COORDINATES:
- x: horizontal (0=left, 960=center, 1920=right)
- y: vertical (0=top/horizon, 1080=bottom/near car)

OUTPUT FORMAT (JSON only, no other text):
{
    "waypoints": [[x0,y0], [x1,y1], [x2,y2], [x3,y3], [x4,y4],
                  [x5,y5], [x6,y6], [x7,y7], [x8,y8], [x9,y9]],
    "target_speed": <km/h>,
    "reasoning": "<brief description>"
}

WAYPOINT RULES:
- Point 0: near car (y ≈ 900-1000)
- Point 9: near horizon (y ≈ 280-350)
- y decreases from point 0 to point 9 (bottom to top)
- x follows the road/lane direction

EXAMPLES:

STRAIGHT ROAD:
{"waypoints": [[960,950], [960,850], [960,750], [960,650], [960,550], [960,470], [960,410], [960,360], [960,320], [960,290]], "target_speed": 30, "reasoning": "Straight road ahead"}

LEFT CURVE:
{"waypoints": [[960,950], [940,850], [910,750], [870,650], [820,550], [770,470], [720,410], [680,360], [645,320], [615,290]], "target_speed": 25, "reasoning": "Road curves left"}

RIGHT CURVE:
{"waypoints": [[960,950], [980,850], [1010,750], [1050,650], [1100,550], [1150,470], [1200,410], [1240,360], [1275,320], [1305,290]], "target_speed": 25, "reasoning": "Road curves right"}

STOP:
{"waypoints": [[960,950], [960,920], [960,900], [960,885], [960,875], [960,868], [960,863], [960,860], [960,858], [960,856]], "target_speed": 0, "reasoning": "Red light or obstacle"}

Follow the lane markings and road edges. Output JSON only."""


# =============================================================================
# CAMERA GEOMETRY
# =============================================================================

class CameraGeometry:
    """Camera projection for 1920x1080 @ 105° FOV."""
    
    def __init__(self, width=1920, height=1080, fov=105):
        self.width = width
        self.height = height
        self.fov = fov
        
        # Focal length from FOV
        fov_rad = math.radians(fov)
        self.focal = width / (2.0 * math.tan(fov_rad / 2.0))
        
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        # Depths for waypoints
        self.depths = CONFIG.WAYPOINT_DEPTHS
    
    def world_to_vehicle(self, world_loc: carla.Location, vehicle_transform: carla.Transform) -> Tuple[float, float, float]:
        """Convert world coordinates to vehicle frame."""
        dx = world_loc.x - vehicle_transform.location.x
        dy = world_loc.y - vehicle_transform.location.y
        dz = world_loc.z - vehicle_transform.location.z
        
        yaw = math.radians(vehicle_transform.rotation.yaw)
        
        x_veh = dx * math.cos(yaw) + dy * math.sin(yaw)
        y_veh = -dx * math.sin(yaw) + dy * math.cos(yaw)
        z_veh = dz
        
        return (x_veh, y_veh, z_veh)
    
    def vehicle_to_pixel(self, x: float, y: float, z: float = 0, 
                         camera_height: float = 1.8, camera_pitch: float = -10.0) -> Optional[Tuple[int, int]]:
        """Project vehicle frame to pixel coordinates."""
        if x <= 0.5:
            return None
        
        # Camera offset from vehicle origin
        x_cam = x - 2.0
        y_cam = y
        z_cam = z - camera_height
        
        # Apply pitch
        pitch_rad = math.radians(camera_pitch)
        x_rot = x_cam * math.cos(pitch_rad) - z_cam * math.sin(pitch_rad)
        z_rot = x_cam * math.sin(pitch_rad) + z_cam * math.cos(pitch_rad)
        
        if x_rot <= 0.1:
            return None
        
        u = self.focal * (-y_cam) / x_rot + self.cx
        v = self.focal * (-z_rot) / x_rot + self.cy
        
        if 0 <= u < self.width and 0 <= v < self.height:
            return (int(u), int(v))
        return None
    
    def pixel_to_vehicle(self, u: float, depth: float) -> Tuple[float, float]:
        """Convert pixel x-coordinate to vehicle frame."""
        x = depth
        y = -((u - self.cx) * depth / self.focal)
        return (x, y)


# =============================================================================
# VLM TRAJECTORY PREDICTOR
# =============================================================================

class VLMPredictor:
    """Async VLM trajectory predictor."""
    
    def __init__(self, config: Config):
        self.config = config
        self.model = ChatOpenAI(
            model=config.VLM_MODEL,
            temperature=config.VLM_TEMPERATURE,
            max_tokens=1024,
            api_key="none",
            base_url=config.VLM_BASE_URL
        )
        
        self.camera_geo = CameraGeometry(
            config.CAMERA_WIDTH, 
            config.CAMERA_HEIGHT,
            config.CAMERA_FOV
        )
        
        # Async prediction
        self.prediction_queue = queue.Queue(maxsize=1)
        self.running = True
        self.thread = threading.Thread(target=self._prediction_loop, daemon=True)
        self.thread.start()
        
        # Latest result
        self.latest_result = None
        self.last_prediction_time = 0
    
    def _encode_image(self, image: Image.Image) -> str:
        buffered = BytesIO()
        image.save(buffered, format="JPEG", quality=85)
        return base64.b64encode(buffered.getvalue()).decode('utf-8')
    
    def _parse_response(self, text: str) -> Optional[Dict]:
        try:
            text = re.sub(r'```json\s*', '', text)
            text = re.sub(r'```\s*', '', text)
            match = re.search(r'\{[\s\S]*\}', text)
            if match:
                return json.loads(match.group())
        except:
            pass
        return None
    
    def _prediction_loop(self):
        """Background prediction thread."""
        while self.running:
            try:
                image, speed = self.prediction_queue.get(timeout=0.1)
                
                img_base64 = self._encode_image(image)
                
                messages = [
                    SystemMessage(content=SYSTEM_PROMPT),
                    HumanMessage(content=[
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}},
                        {"type": "text", "text": f"Current speed: {speed:.1f} km/h. Predict trajectory. JSON only."}
                    ])
                ]
                
                t0 = time.time()
                response = self.model.invoke(messages)
                inference_time = time.time() - t0
                
                result = self._parse_response(response.content)
                
                if result and "waypoints" in result:
                    waypoints_pixel = []
                    for wp in result["waypoints"]:
                        if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                            x = max(0, min(int(wp[0]), self.config.CAMERA_WIDTH - 1))
                            y = max(0, min(int(wp[1]), self.config.CAMERA_HEIGHT - 1))
                            waypoints_pixel.append((x, y))
                    
                    # Convert to vehicle frame
                    waypoints_vehicle = []
                    for i, (u, v) in enumerate(waypoints_pixel):
                        if i < len(self.camera_geo.depths):
                            depth = self.camera_geo.depths[i]
                            x, y = self.camera_geo.pixel_to_vehicle(u, depth)
                            waypoints_vehicle.append((x, y))
                    
                    self.latest_result = {
                        "waypoints_pixel": waypoints_pixel,
                        "waypoints_vehicle": waypoints_vehicle,
                        "target_speed": result.get("target_speed", 15),
                        "reasoning": result.get("reasoning", ""),
                        "inference_time": inference_time,
                        "success": True
                    }
                else:
                    self.latest_result = {
                        "waypoints_pixel": [],
                        "waypoints_vehicle": [],
                        "target_speed": 0,
                        "reasoning": "Parse failed",
                        "inference_time": inference_time,
                        "success": False
                    }
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"VLM error: {e}")
    
    def request_prediction(self, image: Image.Image, speed: float):
        """Request async prediction (non-blocking)."""
        try:
            try:
                self.prediction_queue.get_nowait()
            except queue.Empty:
                pass
            self.prediction_queue.put_nowait((image, speed))
            self.last_prediction_time = time.time()
        except queue.Full:
            pass
    
    def get_latest_result(self) -> Optional[Dict]:
        return self.latest_result
    
    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)


# =============================================================================
# PID CONTROLLER
# =============================================================================

class PIDController:
    """PID controller for steering and speed."""
    
    def __init__(self):
        self.steer_kp = 1.2
        self.steer_ki = 0.01
        self.steer_kd = 0.4
        self.steer_integral = 0.0
        self.steer_prev_error = 0.0
        
        self.speed_kp = 0.5
        self.speed_ki = 0.1
        self.speed_kd = 0.05
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        
        self.wheelbase = 2.875
        self.max_steer_angle = math.radians(70)
    
    def compute_steering(self, waypoints_vehicle: List[Tuple[float, float]], speed: float, dt: float) -> float:
        """Pure Pursuit steering."""
        if not waypoints_vehicle:
            return 0.0
        
        lookahead = max(3.0, min(speed * 0.5, 12.0))
        
        best_wp = waypoints_vehicle[0]
        best_diff = float('inf')
        for wp in waypoints_vehicle:
            diff = abs(wp[0] - lookahead)
            if diff < best_diff:
                best_diff = diff
                best_wp = wp
        
        x, y = best_wp
        L_sq = x*x + y*y
        if L_sq < 0.01:
            return 0.0
        
        curvature = 2.0 * y / L_sq
        steering_angle = math.atan(curvature * self.wheelbase)
        steering = steering_angle / self.max_steer_angle
        
        return float(np.clip(steering, -1.0, 1.0))
    
    def compute_throttle_brake(self, target_speed: float, current_speed: float, dt: float) -> Tuple[float, float]:
        """Speed PID control."""
        error = target_speed - current_speed
        
        self.speed_integral += error * dt
        self.speed_integral = np.clip(self.speed_integral, -10, 10)
        
        derivative = (error - self.speed_prev_error) / max(dt, 0.01)
        self.speed_prev_error = error
        
        output = self.speed_kp * error + self.speed_ki * self.speed_integral + self.speed_kd * derivative
        
        if output >= 0:
            throttle = np.clip(output, 0, 1)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-output, 0, 1)
        
        if target_speed > 1.0 and current_speed < 1.0:
            throttle = max(throttle, 0.5)
        
        return float(throttle), float(brake)
    
    def reset(self):
        self.steer_integral = 0.0
        self.steer_prev_error = 0.0
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0


# =============================================================================
# MAIN APPLICATION
# =============================================================================

class CarlaVLMDriver:
    """Main application integrating CARLA, VLM, and visualization."""
    
    def __init__(self, config: Config):
        self.config = config
        self.camera_geo = CameraGeometry(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FOV)
        
        # CARLA
        self.client = None
        self.world = None
        self.carla_map = None
        self.vehicle = None
        self.camera = None
        
        # State
        self.current_image = None
        self.image_lock = threading.Lock()
        self.speed_kmh = 0.0
        self.control_mode = "autopilot"
        self.vlm_enabled = True
        
        # Ground truth
        self.gt_waypoints_pixel = []
        self.gt_waypoints_vehicle = []
        
        # VLM
        self.vlm = None
        self.pid = PIDController()
        self.last_vlm_request = 0
        
        # Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((config.DISPLAY_WIDTH, config.DISPLAY_HEIGHT))
        pygame.display.set_caption("CARLA + VLM Trajectory Prediction")
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.SysFont('monospace', 24, bold=True)
        self.font_medium = pygame.font.SysFont('monospace', 18)
        self.font_small = pygame.font.SysFont('monospace', 14)
        
        self.frame_count = 0
    
    def connect(self) -> bool:
        """Connect to CARLA."""
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client(self.config.CARLA_HOST, self.config.CARLA_PORT)
            self.client.set_timeout(10.0)
            
            self.world = self.client.get_world()
            self.carla_map = self.world.get_map()
            
            for actor in self.world.get_actors().filter('vehicle.*'):
                actor.destroy()
            for actor in self.world.get_actors().filter('sensor.*'):
                actor.destroy()
            
            print(f"Connected to CARLA, map: {self.carla_map.name}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def spawn_vehicle(self) -> bool:
        """Spawn ego vehicle."""
        try:
            bp_lib = self.world.get_blueprint_library()
            vehicle_bp = bp_lib.find('vehicle.tesla.model3')
            
            spawn_points = self.carla_map.get_spawn_points()
            spawn_point = spawn_points[0]
            
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.vehicle.set_autopilot(True)
            
            print(f"Vehicle spawned at {spawn_point.location}")
            return True
        except Exception as e:
            print(f"Spawn failed: {e}")
            return False
    
    def attach_camera(self) -> bool:
        """Attach RGB camera."""
        try:
            bp_lib = self.world.get_blueprint_library()
            camera_bp = bp_lib.find('sensor.camera.rgb')
            
            camera_bp.set_attribute('image_size_x', str(self.config.CAMERA_WIDTH))
            camera_bp.set_attribute('image_size_y', str(self.config.CAMERA_HEIGHT))
            camera_bp.set_attribute('fov', str(self.config.CAMERA_FOV))
            
            extent = self.vehicle.bounding_box.extent
            camera_transform = carla.Transform(
                carla.Location(x=extent.x, y=0, z=extent.z + 1.0),
                carla.Rotation(pitch=-10, yaw=0, roll=0)
            )
            
            self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            self.camera.listen(self._on_camera_image)
            
            print("Camera attached")
            return True
        except Exception as e:
            print(f"Camera failed: {e}")
            return False
    
    def init_vlm(self):
        """Initialize VLM predictor."""
        print("Initializing VLM...")
        self.vlm = VLMPredictor(self.config)
        print("VLM ready")
    
    def _on_camera_image(self, image: carla.Image):
        """Camera callback."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))[:, :, :3]
        array = array[:, :, ::-1]
        
        with self.image_lock:
            self.current_image = Image.fromarray(array)
    
    def get_ground_truth_trajectory(self) -> Tuple[List[Tuple[int, int]], List[Tuple[float, float]]]:
        """Get ground truth trajectory from CARLA waypoints."""
        vehicle_transform = self.vehicle.get_transform()
        current_wp = self.carla_map.get_waypoint(vehicle_transform.location)
        
        if current_wp is None:
            return [], []
        
        waypoints_pixel = []
        waypoints_vehicle = []
        
        for depth in self.config.WAYPOINT_DEPTHS:
            future_wps = current_wp.next(depth)
            if future_wps:
                wp = future_wps[0]
                x, y, z = self.camera_geo.world_to_vehicle(wp.transform.location, vehicle_transform)
                waypoints_vehicle.append((round(x, 2), round(y, 3)))
                
                pixel = self.camera_geo.vehicle_to_pixel(x, y, z)
                if pixel:
                    waypoints_pixel.append(pixel)
        
        return waypoints_pixel, waypoints_vehicle
    
    def update_state(self):
        """Update vehicle state and ground truth."""
        velocity = self.vehicle.get_velocity()
        self.speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.gt_waypoints_pixel, self.gt_waypoints_vehicle = self.get_ground_truth_trajectory()
    
    def handle_events(self) -> bool:
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                elif event.key == pygame.K_SPACE:
                    if self.control_mode == "autopilot":
                        self.control_mode = "vlm"
                        self.vehicle.set_autopilot(False)
                        self.pid.reset()
                        print("Switched to VLM control")
                    else:
                        self.control_mode = "autopilot"
                        self.vehicle.set_autopilot(True)
                        print("Switched to Autopilot")
                elif event.key == pygame.K_v:
                    self.vlm_enabled = not self.vlm_enabled
                    print(f"VLM prediction: {'ON' if self.vlm_enabled else 'OFF'}")
                elif event.key == pygame.K_r:
                    spawn_points = self.carla_map.get_spawn_points()
                    self.vehicle.set_transform(spawn_points[0])
                    print("Vehicle reset")
                elif event.key == pygame.K_s:
                    self.save_frame()
        return True
    
    def save_frame(self):
        """Save current frame."""
        with self.image_lock:
            if self.current_image is None:
                return
            image = self.current_image.copy()
        
        timestamp = int(time.time() * 1000)
        os.makedirs("saved_frames", exist_ok=True)
        image.save(f"saved_frames/frame_{timestamp}.jpg")
        
        vlm_result = self.vlm.get_latest_result() if self.vlm else None
        annotation = {
            "timestamp": timestamp,
            "speed_kmh": self.speed_kmh,
            "ground_truth_pixel": self.gt_waypoints_pixel,
            "vlm_prediction": vlm_result
        }
        with open(f"saved_frames/frame_{timestamp}.json", "w") as f:
            json.dump(annotation, f, indent=2)
        
        print(f"Saved frame_{timestamp}")
    
    def apply_vlm_control(self, dt: float):
        """Apply VLM-based control."""
        if not self.vlm:
            return
            
        vlm_result = self.vlm.get_latest_result()
        
        if vlm_result and vlm_result.get("success") and vlm_result.get("waypoints_vehicle"):
            steering = self.pid.compute_steering(vlm_result["waypoints_vehicle"], self.speed_kmh / 3.6, dt)
            target_speed = vlm_result.get("target_speed", 15)
            throttle, brake = self.pid.compute_throttle_brake(target_speed, self.speed_kmh, dt)
            
            control = carla.VehicleControl()
            control.steer = steering
            control.throttle = throttle
            control.brake = brake
            self.vehicle.apply_control(control)
        else:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.3
            self.vehicle.apply_control(control)
    
    def draw_trajectory(self, surface: pygame.Surface, waypoints: List[Tuple[int, int]], 
                        color: Tuple[int, int, int], label_prefix: str = ""):
        """Draw trajectory."""
        if not waypoints:
            return
        
        if len(waypoints) >= 2:
            pygame.draw.lines(surface, color, False, waypoints, 3)
        
        for i, (x, y) in enumerate(waypoints):
            point_color = (0, 255, 0) if i == 0 else color
            pygame.draw.circle(surface, point_color, (x, y), 8)
            pygame.draw.circle(surface, (255, 255, 255), (x, y), 8, 2)
            
            label = f"{label_prefix}{i}" if label_prefix else str(i)
            text = self.font_small.render(label, True, (255, 255, 255))
            surface.blit(text, (x + 10, y - 8))
    
    def draw_info_panel(self):
        """Draw information panel."""
        x, y = 20, 20
        
        def draw_text(text, color=(255, 255, 255), font=None):
            nonlocal y
            font = font or self.font_medium
            surface = font.render(text, True, color)
            bg = pygame.Surface((surface.get_width() + 10, surface.get_height() + 4))
            bg.set_alpha(180)
            bg.fill((0, 0, 0))
            self.screen.blit(bg, (x - 5, y - 2))
            self.screen.blit(surface, (x, y))
            y += font.get_height() + 4
        
        draw_text("CARLA + VLM Trajectory", (100, 200, 255), self.font_large)
        y += 10
        
        mode_color = (0, 255, 0) if self.control_mode == "autopilot" else (255, 165, 0)
        draw_text(f"Mode: {self.control_mode.upper()}", mode_color)
        
        vlm_color = (0, 255, 0) if self.vlm_enabled else (128, 128, 128)
        draw_text(f"VLM: {'ON' if self.vlm_enabled else 'OFF'}", vlm_color)
        draw_text(f"Speed: {self.speed_kmh:.1f} km/h")
        
        if self.vlm:
            vlm_result = self.vlm.get_latest_result()
            if vlm_result:
                y += 10
                draw_text("─── VLM Prediction ───", (150, 150, 150))
                status_color = (0, 255, 0) if vlm_result.get("success") else (255, 0, 0)
                draw_text(f"Status: {'OK' if vlm_result.get('success') else 'FAIL'}", status_color)
                draw_text(f"Target: {vlm_result.get('target_speed', 0)} km/h")
                draw_text(f"Inference: {vlm_result.get('inference_time', 0):.2f}s")
                
                reasoning = vlm_result.get("reasoning", "")
                if reasoning:
                    reasoning = reasoning[:40] + "..." if len(reasoning) > 40 else reasoning
                    draw_text(f"Reason: {reasoning}", (200, 200, 200), self.font_small)
        
        y = self.config.DISPLAY_HEIGHT - 140
        draw_text("─── Controls ───", (100, 100, 100))
        draw_text("SPACE: Toggle Autopilot/VLM", (150, 150, 150), self.font_small)
        draw_text("V: Toggle VLM prediction", (150, 150, 150), self.font_small)
        draw_text("S: Save frame  |  R: Reset  |  ESC: Quit", (150, 150, 150), self.font_small)
    
    def draw_legend(self):
        """Draw trajectory legend."""
        x = self.config.DISPLAY_WIDTH - 220
        y = 20
        
        bg = pygame.Surface((210, 80))
        bg.set_alpha(200)
        bg.fill((0, 0, 0))
        self.screen.blit(bg, (x - 10, y - 10))
        
        pygame.draw.circle(self.screen, (0, 255, 255), (x, y + 15), 6)
        pygame.draw.line(self.screen, (0, 255, 255), (x + 10, y + 15), (x + 40, y + 15), 3)
        text = self.font_small.render("Ground Truth (CARLA)", True, (0, 255, 255))
        self.screen.blit(text, (x + 50, y + 8))
        
        pygame.draw.circle(self.screen, (255, 100, 0), (x, y + 45), 6)
        pygame.draw.line(self.screen, (255, 100, 0), (x + 10, y + 45), (x + 40, y + 45), 3)
        text = self.font_small.render("VLM Prediction", True, (255, 100, 0))
        self.screen.blit(text, (x + 50, y + 38))
    
    def render(self):
        """Render frame."""
        self.screen.fill((30, 30, 30))
        
        with self.image_lock:
            if self.current_image is not None:
                img_str = self.current_image.tobytes()
                img_surface = pygame.image.fromstring(img_str, self.current_image.size, 'RGB')
                self.screen.blit(img_surface, (0, 0))
        
        # Ground truth (CYAN)
        if self.gt_waypoints_pixel:
            self.draw_trajectory(self.screen, self.gt_waypoints_pixel, (0, 255, 255), "")
        
        # VLM prediction (ORANGE)
        if self.vlm_enabled and self.vlm:
            vlm_result = self.vlm.get_latest_result()
            if vlm_result and vlm_result.get("waypoints_pixel"):
                self.draw_trajectory(self.screen, vlm_result["waypoints_pixel"], (255, 100, 0), "")
        
        self.draw_info_panel()
        self.draw_legend()
        pygame.display.flip()
    
    def run(self):
        """Main loop."""
        print("\n" + "="*60)
        print("Controls:")
        print("  SPACE - Toggle Autopilot/VLM control")
        print("  V     - Toggle VLM prediction ON/OFF")
        print("  S     - Save frame")
        print("  R     - Reset vehicle")
        print("  ESC   - Quit")
        print("="*60 + "\n")
        
        running = True
        last_time = time.time()
        
        try:
            while running:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                self.world.tick()
                self.frame_count += 1
                
                running = self.handle_events()
                self.update_state()
                
                # VLM prediction request
                if self.vlm_enabled and self.vlm and current_time - self.last_vlm_request >= self.config.VLM_INTERVAL:
                    with self.image_lock:
                        if self.current_image is not None:
                            self.vlm.request_prediction(self.current_image.copy(), self.speed_kmh)
                            self.last_vlm_request = current_time
                
                # Control
                if self.control_mode == "vlm":
                    self.apply_vlm_control(dt)
                
                self.render()
                self.clock.tick(30)
                
        except KeyboardInterrupt:
            print("\nInterrupted")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources."""
        print("Cleaning up...")
        
        if self.vlm:
            self.vlm.stop()
        
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
        if self.vehicle:
            self.vehicle.destroy()
        
        pygame.quit()
        print("Done")


# =============================================================================
# MAIN
# =============================================================================

def main():
    config = Config()
    driver = CarlaVLMDriver(config)
    
    if not driver.connect():
        return
    
    if not driver.spawn_vehicle():
        return
    
    if not driver.attach_camera():
        return
    
    driver.init_vlm()
    
    print("Waiting for camera...")
    time.sleep(1.0)
    
    driver.run()


if __name__ == "__main__":
    main()