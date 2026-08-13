#!/usr/bin/env python3
"""
Dual-Stage VLM Control Driver

Stage 1 (Base VLM): slower scene understanding -> navigation intent.
Stage 2 (LoRA VLM): faster control prediction conditioned on nav intent.
"""

import argparse
import math
import signal
import time
from typing import Optional, Tuple

import carla
import numpy as np

try:
	import pygame
	PYGAME_AVAILABLE = True
except ImportError:
	PYGAME_AVAILABLE = False
	print("pygame not available - running in headless mode")

from route_builder import RouteBuilder
from vlm_control_inference import VLMControlPredictor
from control_navigation import get_nav_prompt


# =============================================================================
# CONFIGURATION
# =============================================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 480
CAMERA_FOV = 90

NAV_COMMANDS = [
	"follow_lane",
	"slight_left",
	"slight_right",
	"turn_left",
	"turn_right",
	"sharp_turn_left",
	"sharp_turn_right",
	"merge_left",
	"merge_right",
	"intersection_approach",
	"u_turn",
]

OFF_ROUTE_THRESHOLD_M = 5.0

VLM_ENDPOINTS = {
	"local": {
		"url": "https://vishwanaths-mac-mini.woodpecker-bluegill.ts.net/v1",
		"model": "unsloth/Qwen3-VL-2B-Instruct-bnb-4bit",
		"lora": "driver",
	},
	"cloud": {
		"url": "https://1b3565c4dfc4.woodpecker-bluegill.ts.net/v1",
		"model": "unsloth/Qwen3-VL-2B-Instruct-bnb-4bit",
		"lora": "driver",
	},
}


# =============================================================================
# SENSOR DATA
# =============================================================================

class SensorData:
	def __init__(self):
		self.rgb_image = None
		self.timestamp = 0


sensor_data = SensorData()


def process_rgb(image):
	array = np.frombuffer(image.raw_data, dtype=np.uint8)
	array = array.reshape((image.height, image.width, 4))[:, :, :3]
	sensor_data.rgb_image = array[:, :, ::-1].copy()
	sensor_data.timestamp = image.timestamp


# =============================================================================
# NAVIGATION PREDICTOR
# =============================================================================

class VLMNavigationPredictor:
	"""
	Base VLM predictor for navigation intent.
	Outputs a single token from NAV_COMMANDS.
	"""

	SYSTEM_INSTRUCTION = """You are an autonomous driving assistant.
Given a front camera image and current speed, output ONE navigation token from this list:
{nav_list}

Output ONLY the token. No extra text.
"""

	def __init__(self,
				 base_url: str,
				 model: str,
				 api_key: str = "not-needed"):
		self.base_url = base_url
		self.model = model
		self.api_key = api_key
		self.client = None
		self._init_client()

	def _init_client(self):
		try:
			from openai import OpenAI
			self.client = OpenAI(
				base_url=self.base_url,
				api_key=self.api_key
			)
			print(f"Base VLM client initialized: {self.base_url}")
		except ImportError:
			print("Warning: openai package not installed. Run: pip install openai")
			self.client = None

	def _image_to_base64(self, image: np.ndarray) -> str:
		from PIL import Image
		from io import BytesIO
		import base64

		if isinstance(image, np.ndarray):
			pil_image = Image.fromarray(image)
		else:
			pil_image = image

		buffer = BytesIO()
		pil_image.save(buffer, format="JPEG", quality=85)
		return base64.b64encode(buffer.getvalue()).decode("utf-8")

	def predict_navigation(self,
						   image: np.ndarray,
						   speed_kmh: float,
						   context: str = "") -> Tuple[str, float]:
		if self.client is None:
			return "follow_lane", 0.0

		image_b64 = self._image_to_base64(image)
		nav_list = ", ".join(NAV_COMMANDS)
		system_text = self.SYSTEM_INSTRUCTION.format(nav_list=nav_list)

		user_text = f"""Current Status:
  - Speed: {speed_kmh:.1f} km/h
  - Context: {context}
"""

		messages = [
			{
				"role": "system",
				"content": [
					{"type": "text", "text": system_text}
				]
			},
			{
				"role": "user",
				"content": [
					{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
					{"type": "text", "text": user_text},
				]
			}
		]

		start_time = time.time()
		try:
			response = self.client.chat.completions.create(
				model=self.model,
				messages=messages,
				max_tokens=10,
			)
			inference_time = time.time() - start_time
			response_text = response.choices[0].message.content.strip().lower()

			for token in NAV_COMMANDS:
				if token in response_text:
					return token, inference_time

			return "follow_lane", inference_time
		except Exception as exc:
			print(f"Base VLM API error: {exc}")
			return "follow_lane", time.time() - start_time


# =============================================================================
# DRIVER
# =============================================================================

class DualStageDriver:
	def __init__(self,
				 base_predictor: VLMNavigationPredictor,
				 lora_predictor: VLMControlPredictor,
				 base_hz: float = 1.0,
				 lora_hz: float = 4.0,
				 open_map_prompt: str = "Follow traffic rules and drive safely"):
		self.base_predictor = base_predictor
		self.lora_predictor = lora_predictor
		self.base_interval = 1.0 / max(base_hz, 0.1)
		self.lora_interval = 1.0 / max(lora_hz, 0.1)
		self.open_map_prompt = open_map_prompt

		self.last_base_time = 0.0
		self.last_lora_time = 0.0
		self.last_nav_command = "follow_lane"
		self.last_control = {"steer": 0.0, "throttle": 0.0, "brake": 0.0}

		self.base_inference_count = 0
		self.base_total_time = 0.0
		self.lora_inference_count = 0
		self.lora_total_time = 0.0

		self.turn_persist_frames = 3
		self._turn_left_count = 0
		self._turn_right_count = 0

	def smooth_nav_command(self, nav_command: Optional[str]) -> str:
		if nav_command is None:
			return self.last_nav_command

		if nav_command == "turn_left":
			self._turn_left_count += 1
			self._turn_right_count = 0
			if self._turn_left_count < self.turn_persist_frames:
				return "slight_left"
			return "turn_left"

		if nav_command == "turn_right":
			self._turn_right_count += 1
			self._turn_left_count = 0
			if self._turn_right_count < self.turn_persist_frames:
				return "slight_right"
			return "turn_right"

		if nav_command in ("slight_left", "slight_right", "follow_lane"):
			self._turn_left_count = 0
			self._turn_right_count = 0
			return nav_command

		return nav_command

	@staticmethod
	def clamp_nav_off_route(nav_command: str) -> str:
		if nav_command in ("turn_left", "sharp_turn_left"):
			return "slight_left"
		if nav_command in ("turn_right", "sharp_turn_right"):
			return "slight_right"
		if nav_command == "u_turn":
			return "follow_lane"
		return nav_command

	def maybe_update_base_nav(self,
							  image: np.ndarray,
							  speed_kmh: float,
							  context: str) -> Tuple[str, bool]:
		now = time.time()
		if (now - self.last_base_time) < self.base_interval:
			return self.last_nav_command, False

		nav_command, infer_time = self.base_predictor.predict_navigation(
			image,
			speed_kmh,
			context=context
		)
		nav_command = self.smooth_nav_command(nav_command)

		self.last_nav_command = nav_command
		self.last_base_time = now
		self.base_inference_count += 1
		self.base_total_time += infer_time

		return nav_command, True

	def maybe_get_lora_control(self,
							   image: np.ndarray,
							   speed_kmh: float,
							   navigation: str) -> Tuple[carla.VehicleControl, str]:
		now = time.time()
		if (now - self.last_lora_time) < self.lora_interval:
			control = carla.VehicleControl(
				steer=self.last_control["steer"],
				throttle=self.last_control["throttle"],
				brake=self.last_control["brake"],
			)
			return control, "CACHED"

		control_dict, infer_time = self.lora_predictor.predict(
			image,
			speed_kmh=speed_kmh,
			navigation=navigation
		)

		self.last_lora_time = now
		self.lora_inference_count += 1
		self.lora_total_time += infer_time

		if control_dict is None:
			control = carla.VehicleControl(
				steer=self.last_control["steer"],
				throttle=self.last_control["throttle"],
				brake=self.last_control["brake"],
			)
			return control, "FAILED"

		self.last_control = control_dict
		control = carla.VehicleControl(
			steer=control_dict["steer"],
			throttle=control_dict["throttle"],
			brake=control_dict["brake"],
		)
		return control, "NEW"

	def get_stats(self) -> Tuple[float, float]:
		base_avg = self.base_total_time / max(self.base_inference_count, 1)
		lora_avg = self.lora_total_time / max(self.lora_inference_count, 1)
		return base_avg, lora_avg


# =============================================================================
# MAIN
# =============================================================================

def run_dual_stage_drive(
	mode: str,
	base_url: str,
	base_model: str,
	lora_url: str,
	lora_model: str,
	lora_adapter: str,
	base_hz: float,
	lora_hz: float,
	route_file: str = None,
	routes_dir: str = None,
	loops: int = 1,
	open_map_prompt: str = "Follow traffic rules and drive safely",
	use_display: bool = True,
):
	display = None
	clock = None
	font = None
	if use_display and PYGAME_AVAILABLE:
		pygame.init()
		display = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT))
		pygame.display.set_caption("Dual-Stage VLM Control")
		clock = pygame.time.Clock()
		font = pygame.font.SysFont("monospace", 14)

	print(f"\n{'='*70}")
	print("DUAL-STAGE VLM CONTROL")
	print(f"{'='*70}")
	print(f"Mode: {mode}")
	print(f"Base VLM: {base_url} | {base_model}")
	print(f"LoRA VLM: {lora_url} | {lora_model} | {lora_adapter}")
	print(f"Base Hz: {base_hz} | LoRA Hz: {lora_hz}")
	print(f"{'='*70}\n")

	route_builder = None
	if mode == "route":
		route_builder = RouteBuilder(
			routes_dir=routes_dir,
			route_file=route_file,
			waypoint_distances=[3, 6, 9, 12, 15, 18, 21, 24, 27, 30]
		)
		print(f"Loaded {route_builder.get_route_count()} routes\n")

	print("Connecting to CARLA...")
	client = carla.Client("localhost", 2000)
	client.set_timeout(10.0)
	world = client.load_world("Town10HD")
	bp_lib = world.get_blueprint_library()

	base_predictor = VLMNavigationPredictor(
		base_url=base_url,
		model=base_model,
	)
	lora_predictor = VLMControlPredictor(
		base_url=lora_url,
		model=lora_model,
		lora_adapter=lora_adapter,
	)

	driver = DualStageDriver(
		base_predictor=base_predictor,
		lora_predictor=lora_predictor,
		base_hz=base_hz,
		lora_hz=lora_hz,
		open_map_prompt=open_map_prompt,
	)

	actors = []
	vehicle = None
	camera = None
	reset_transform = None

	current_route_idx = 0
	current_route_loop = 0
	routes_completed = 0
	current_route = None

	shutdown_requested = False
	def signal_handler(sig, frame):
		nonlocal shutdown_requested
		print("\n\nShutdown requested...")
		shutdown_requested = True

	signal.signal(signal.SIGINT, signal_handler)

	try:
		vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]

		if mode == "route" and route_builder:
			current_route = route_builder.get_route(current_route_idx)
			spawn_info = route_builder.get_spawn_transform(current_route)
			spawn_transform = carla.Transform(
				carla.Location(**spawn_info["location"]),
				carla.Rotation(**spawn_info["rotation"])
			)
			print(f"Spawning at route: {current_route.name}")
		else:
			spawn_points = world.get_map().get_spawn_points()
			spawn_transform = spawn_points[0] if spawn_points else carla.Transform()
			print("Spawning at map default")

		vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
		actors.append(vehicle)
		reset_transform = spawn_transform

		cam_bp = bp_lib.find("sensor.camera.rgb")
		cam_bp.set_attribute("image_size_x", str(CAMERA_WIDTH))
		cam_bp.set_attribute("image_size_y", str(CAMERA_HEIGHT))
		cam_bp.set_attribute("fov", str(CAMERA_FOV))

		cam_transform = carla.Transform(
			carla.Location(x=2.0, y=0.0, z=1.8),
			carla.Rotation(pitch=-15, yaw=0, roll=0)
		)

		camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
		actors.append(camera)
		camera.listen(process_rgb)

		time.sleep(1.0)
		print("Starting dual-stage control...\n")

		frame = 0
		total_routes = route_builder.get_route_count() * loops if mode == "route" else 0

		while not shutdown_requested:
			if display:
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						shutdown_requested = True
					elif event.type == pygame.KEYDOWN:
						if event.key in (pygame.K_ESCAPE, pygame.K_q):
							shutdown_requested = True
						elif event.key == pygame.K_r and vehicle is not None and reset_transform is not None:
							vehicle.set_transform(reset_transform)
							vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
							vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
							time.sleep(0.2)

			transform = vehicle.get_transform()
			velocity = vehicle.get_velocity()
			speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
			off_route = False

			if mode == "route" and current_route and route_builder:
				closest_idx = route_builder.find_closest_checkpoint(
					current_route,
					transform.location
				)
				closest_cp = current_route.checkpoints[closest_idx]
				dist_to_route = math.sqrt(
					(transform.location.x - closest_cp.x) ** 2
					+ (transform.location.y - closest_cp.y) ** 2
				)
				off_route = dist_to_route > OFF_ROUTE_THRESHOLD_M

				if route_builder.is_route_complete(current_route, transform.location, threshold_meters=5.0):
					routes_completed += 1
					current_route_loop += 1
					print(f"\nRoute complete: {current_route.name} ({current_route_loop}/{loops})")

					if current_route_loop >= loops:
						current_route_idx = (current_route_idx + 1) % route_builder.get_route_count()
						current_route_loop = 0

						if routes_completed < total_routes:
							current_route = route_builder.get_route(current_route_idx)
							print(f"Starting route: {current_route.name}")

					spawn_info = route_builder.get_spawn_transform(current_route)
					new_transform = carla.Transform(
						carla.Location(**spawn_info["location"]),
						carla.Rotation(**spawn_info["rotation"])
					)
					vehicle.set_transform(new_transform)
					reset_transform = new_transform
					time.sleep(0.5)
					continue

			if sensor_data.rgb_image is None:
				world.tick()
				time.sleep(0.033)
				continue

			context = open_map_prompt if mode == "open" else "Route following"
			nav_cmd, base_updated = driver.maybe_update_base_nav(
				sensor_data.rgb_image,
				speed_kmh,
				context=context
			)
			if off_route:
				nav_cmd = driver.clamp_nav_off_route(nav_cmd)
			nav_prompt = get_nav_prompt(nav_cmd)

			control, status = driver.maybe_get_lora_control(
				sensor_data.rgb_image,
				speed_kmh,
				navigation=nav_prompt
			)

			vehicle.apply_control(control)

			if display and sensor_data.rgb_image is not None:
				display.fill((0, 0, 0))
				surface = pygame.surfarray.make_surface(sensor_data.rgb_image.swapaxes(0, 1))
				display.blit(surface, (0, 0))

				base_avg, lora_avg = driver.get_stats()
				panel_x = CAMERA_WIDTH + 10
				panel_y = 10
				panel_w = DISPLAY_WIDTH - CAMERA_WIDTH - 20
				panel_h = DISPLAY_HEIGHT - 20
				panel_rect = pygame.Rect(panel_x - 5, panel_y - 5, panel_w + 10, panel_h + 10)
				pygame.draw.rect(display, (0, 0, 0, 180), panel_rect)

				base_message = f"Base: {nav_cmd}"
				lora_message = (
					f"LoRA: S={control.steer:+.2f} T={control.throttle:.2f} B={control.brake:.2f}"
				)

				info_lines = [
					"CHAT",
					base_message,
					lora_message,
					None,
					"BASE VLM",
					f"Update: {'NEW' if base_updated else 'CACHED'}",
					f"Avg: {base_avg:.2f}s",
					None,
					"LORA VLM",
					f"Status: {status}",
					f"Avg: {lora_avg:.2f}s",
					None,
					"STATE",
					f"Speed: {speed_kmh:.1f} km/h",
					f"Off-route: {off_route}",
					"Controls: R=reset, Q/Esc=quit",
				]

				y_offset = panel_y
				for line in info_lines:
					if line is None:
						y_offset += 10
						continue
					text = font.render(line, True, (255, 255, 255))
					display.blit(text, (panel_x, y_offset))
					y_offset += 18

				pygame.display.flip()
				clock.tick(30)
			else:
				world.tick()
				time.sleep(0.033)

			if frame % 30 == 0:
				base_avg, lora_avg = driver.get_stats()
				print(
					f"\rNav: {nav_cmd} | Speed: {speed_kmh:.0f}km/h | "
					f"Base: {base_avg:.2f}s | LoRA: {lora_avg:.2f}s",
					end=""
				)

			frame += 1

	finally:
		print("\nCleaning up...")
		for actor in actors:
			if actor is not None:
				actor.destroy()

		if display:
			pygame.quit()


# =============================================================================
# CLI
# =============================================================================

if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Dual-stage base+LoRA VLM control",
		formatter_class=argparse.RawDescriptionHelpFormatter,
	)

	parser.add_argument("--mode", type=str, required=True, choices=["open", "route"])
	parser.add_argument("--vlm-preset", type=str, default="cloud", choices=["local", "cloud"])

	parser.add_argument("--base-url", type=str, help="Base VLM URL (overrides preset)")
	parser.add_argument("--base-model", type=str, help="Base VLM model (overrides preset)")

	parser.add_argument("--lora-url", type=str, help="LoRA VLM URL (overrides preset)")
	parser.add_argument("--lora-model", type=str, help="LoRA base model (overrides preset)")
	parser.add_argument("--lora-adapter", type=str, help="LoRA adapter name (overrides preset)")

	parser.add_argument("--base-hz", type=float, default=1.0)
	parser.add_argument("--lora-hz", type=float, default=4.0)

	parser.add_argument("--route", type=str, help="Route JSON file (route mode)")
	parser.add_argument("--routes", type=str, help="Routes directory (route mode)")
	parser.add_argument("--loops", type=int, default=1, help="Loops per route")

	parser.add_argument("--prompt", type=str, default="Follow traffic rules and drive safely")
	parser.add_argument("--no-display", action="store_true")

	args = parser.parse_args()

	if args.mode == "route" and not args.route and not args.routes:
		parser.error("Route mode requires --route or --routes")

	preset = VLM_ENDPOINTS[args.vlm_preset]
	base_url = args.base_url or preset["url"]
	base_model = args.base_model or preset["model"]
	lora_url = args.lora_url or preset["url"]
	lora_model = args.lora_model or preset["model"]
	lora_adapter = args.lora_adapter or preset.get("lora")

	run_dual_stage_drive(
		mode=args.mode,
		base_url=base_url,
		base_model=base_model,
		lora_url=lora_url,
		lora_model=lora_model,
		lora_adapter=lora_adapter,
		base_hz=args.base_hz,
		lora_hz=args.lora_hz,
		route_file=args.route,
		routes_dir=args.routes,
		loops=args.loops,
		open_map_prompt=args.prompt,
		use_display=not args.no_display,
	)
