#!/usr/bin/env python3
"""
VLM Inference Module for CARLA Trajectory Prediction

Communicates with a local VLM server (OpenAI API format) at localhost:1234
to predict driving trajectories from camera images.

Usage:
    from vlm_inference import VLMTrajectoryPredictor
    
    predictor = VLMTrajectoryPredictor()
    trajectory_2d = predictor.predict(camera_image)
    # Returns: [[x, y, distance], [x, y, distance], ...]
"""

import json
import re
import base64
import numpy as np
from PIL import Image
from typing import List, Tuple, Optional
import time
from io import BytesIO


class VLMTrajectoryPredictor:
    """
    VLM-based trajectory prediction using OpenAI-compatible API.
    Connects to local server at localhost:1234.
    """

    PROMPT_TEMPLATE = """Current Status:
- Speed: {speed} km/h
- Navigation: {navigation}

Predict the driving trajectory as pixel coordinates with distance.
Output format: [[x1,y1,d1], [x2,y2,d2], [x3,y3,d3], ...]"""
    
    def __init__(self,
                 base_url: str = "http://localhost:1234/v1",
                 api_key: str = "not-needed",
                 model: str = "vla-2b",
                 lora_adapter: str = None):
        """
        Initialize VLM predictor with OpenAI-compatible API.

        Args:
            base_url: API endpoint (default: localhost:1234)
            api_key: API key (usually not needed for local)
            model: Base model name (e.g., "unsloth/Qwen3-VL-2B-Instruct-bnb-4bit")
            lora_adapter: LoRA adapter name (e.g., "driver") - optional
        """
        self.base_url = base_url
        self.api_key = api_key
        self.model = model
        self.lora_adapter = lora_adapter
        self.client = None
        self._init_client()

        if self.lora_adapter:
            print(f"  LoRA adapter: {self.lora_adapter}")
        
    def _init_client(self):
        """Initialize OpenAI client."""
        try:
            from openai import OpenAI
            self.client = OpenAI(
                base_url=self.base_url,
                api_key=self.api_key
            )
            print(f"VLM client initialized: {self.base_url}")
        except ImportError:
            print("Warning: openai package not installed. Run: pip install openai")
            self.client = None
    
    def _image_to_base64(self, image: np.ndarray) -> str:
        """Convert numpy image to base64 string."""
        if isinstance(image, np.ndarray):
            pil_image = Image.fromarray(image)
        else:
            pil_image = image
        
        buffer = BytesIO()
        pil_image.save(buffer, format="JPEG", quality=85)
        return base64.b64encode(buffer.getvalue()).decode('utf-8')
    
    def predict(self, image: np.ndarray, temperature: float = 0.3,
                current_speed: float = None, navigation: str = None) -> Tuple[List[List[float]], float]:
        """
        Predict trajectory from camera image.

        Args:
            image: RGB image as numpy array (H, W, 3)
            temperature: Sampling temperature (lower = more deterministic)
            current_speed: Current vehicle speed in km/h (default: 30.0)
            navigation: Navigation command e.g. "lane_keeping", "turn_left", "turn_right" (default: "lane_keeping")

        Returns:
            (trajectory, inference_time) where trajectory is [[pixel_x, pixel_y], ...]
        """
        if self.client is None:
            return self._mock_predict()

        # Set defaults
        if current_speed is None:
            current_speed = 30.0
        if navigation is None:
            navigation = "lane_keeping"

        # Convert image to base64
        image_b64 = self._image_to_base64(image)

        # Format prompt with context
        prompt_text = self.PROMPT_TEMPLATE.format(
            speed=current_speed,
            navigation=navigation
        )

        # Build message content matching OpenAI API format
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_b64}"
                        }
                    },
                    {
                        "type": "text",
                        "text": prompt_text
                    }
                ]
            }
        ]

        # Debug: Print payload structure (without base64 image data)
        debug_content = [
            {"type": "image_url", "image_url": {"url": "[IMAGE_DATA]"}},
            {"type": "text", "text": prompt_text}
        ]
        print(f"\n[VLM PAYLOAD] Messages: {json.dumps([{'role': 'user', 'content': debug_content}], indent=2)}")

        # Call API
        start_time = time.time()
        try:
            # In vLLM OpenAI API, LoRA adapters are specified using the "model" parameter
            # The server is started with: --lora-modules {adapter_name}={adapter_path}
            # Then you use model={adapter_name} in the request
            model_name = self.lora_adapter if self.lora_adapter else self.model

            response = self.client.chat.completions.create(
                model=model_name,
                messages=messages,
                max_tokens=256,
                temperature=temperature,
            )
            inference_time = time.time() - start_time
            
            # Extract response text
            response_text = response.choices[0].message.content
            
            # Parse trajectory
            trajectory = self._parse_trajectory(response_text)
            
            return trajectory, inference_time
            
        except Exception as e:
            print(f"VLM API error: {e}")
            return [], time.time() - start_time
    
    def _parse_trajectory(self, response: str) -> List[List[float]]:
        """
        Parse trajectory from model response.

        Expected format: [[x1, y1, d1], [x2, y2, d2], ...]
        Also accepts legacy format: [[x1, y1], [x2, y2], ...]
        """
        # Find JSON array pattern
        match = re.search(r'\[\[[\d\s,.\-]+\]\]', response.replace('\n', ''))
        if match:
            try:
                trajectory = json.loads(match.group())
                # Validate format
                if isinstance(trajectory, list) and len(trajectory) > 0:
                    if isinstance(trajectory[0], list) and len(trajectory[0]) >= 2:
                        return trajectory
            except json.JSONDecodeError:
                pass

        # Fallback: try to extract numbers
        numbers = re.findall(r'[\d.]+', response)
        if len(numbers) >= 6:  # At least 2 waypoints (x, y, d triplets)
            trajectory = []
            # Try triplets first (x, y, distance)
            for i in range(0, len(numbers) - 2, 3):
                try:
                    x = float(numbers[i])
                    y = float(numbers[i + 1])
                    d = float(numbers[i + 2])
                    trajectory.append([x, y, d])
                except (ValueError, IndexError):
                    break
            if trajectory and len(trajectory) >= 2:
                return trajectory

        # Fallback to pairs (legacy format)
        if len(numbers) >= 4:
            trajectory = []
            for i in range(0, len(numbers) - 1, 2):
                try:
                    x = float(numbers[i])
                    y = float(numbers[i + 1])
                    trajectory.append([x, y])
                except (ValueError, IndexError):
                    break
            if trajectory:
                return trajectory

        return []
    
    def _mock_predict(self) -> Tuple[List[List[float]], float]:
        """Return mock trajectory for testing without model."""
        # Simulated center-line trajectory (9 waypoints with distance, matching training format)
        mock_trajectory = [
            [320, 450, 6],
            [320, 399, 9],
            [320, 384, 12],
            [320, 371, 15],
            [320, 363, 18],
            [320, 357, 21],
            [320, 352, 24],
            [320, 349, 27],
            [320, 347, 30]
        ]
        return mock_trajectory, 0.1


class AsyncVLMPredictor:
    """
    Asynchronous VLM predictor with buffering for real-time driving.

    Handles VLM latency (2-3s) by:
    1. Running inference in background thread
    2. Returning last valid prediction until new one ready
    """

    def __init__(self,
                 base_url: str = "https://vish2kber--vlm-inference-serve.modal.run/v1",
                 api_key: str = "not-needed",
                 model: str = "local-model",
                 lora_adapter: str = None):
        import threading
        import queue

        self.predictor = VLMTrajectoryPredictor(base_url, api_key, model, lora_adapter)

        # Threading
        self._input_queue = queue.Queue(maxsize=1)
        self._output_queue = queue.Queue(maxsize=1)
        self._running = True
        self._thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._thread.start()

        # State
        self._last_trajectory = []
        self._last_inference_time = 0
        self._pending = False

    def _inference_loop(self):
        """Background inference loop."""
        while self._running:
            try:
                data = self._input_queue.get(timeout=0.1)
                image = data['image']
                current_speed = data.get('current_speed')
                navigation = data.get('navigation')

                trajectory, inf_time = self.predictor.predict(
                    image,
                    current_speed=current_speed,
                    navigation=navigation
                )

                # Clear old output and put new one
                try:
                    self._output_queue.get_nowait()
                except:
                    pass
                self._output_queue.put((trajectory, inf_time))

            except:
                continue

    def submit_image(self, image: np.ndarray, current_speed: float = None, navigation: str = None):
        """
        Submit image for async inference.

        Args:
            image: RGB image as numpy array (H, W, 3)
            current_speed: Current vehicle speed in km/h (optional context)
            navigation: Navigation command e.g. "lane_keeping", "turn_left", "turn_right" (optional context)
        """
        try:
            # Clear old pending request
            try:
                self._input_queue.get_nowait()
            except:
                pass
            self._input_queue.put_nowait({
                'image': image.copy(),
                'current_speed': current_speed,
                'navigation': navigation
            })
            self._pending = True
        except:
            pass

    def get_trajectory(self) -> Tuple[List[List[float]], float, bool]:
        """
        Get latest trajectory prediction.

        Returns:
            (trajectory, inference_time, is_new)
        """
        # Check for new result
        try:
            trajectory, inf_time = self._output_queue.get_nowait()
            self._last_trajectory = trajectory
            self._last_inference_time = inf_time
            self._pending = False
            return trajectory, inf_time, True
        except:
            # Return cached
            return self._last_trajectory, self._last_inference_time, False

    def stop(self):
        """Stop background thread."""
        self._running = False
        self._thread.join(timeout=1.0)


def trajectory_to_waypoints_2d(trajectory: List[List[float]]) -> List:
    """
    Convert VLM trajectory output to Waypoint2D objects.

    Args:
        trajectory: [[x1, y1, d1], [x2, y2, d2], ...] or [[x1, y1], [x2, y2], ...]

    Returns:
        List of Waypoint2D objects (only x, y used, distance ignored if present)
    """
    from traj_planner import Waypoint2D

    waypoints = []
    for point in trajectory:
        if len(point) >= 2:
            u, v = point[0], point[1]
            # Ignore distance (point[2]) if present
            waypoints.append(Waypoint2D(u=float(u), v=float(v)))

    return waypoints


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    print("Testing VLM inference module...")
    print("Connecting to https://vish2kber--vlm-inference-serve.modal.run/v1...")
    
    # Test with API
    predictor = VLMTrajectoryPredictor(
        base_url="https://vish2kber--vlm-inference-serve.modal.run/v1",
        model="VishwanathAS/qwen-vl-drive"  # Change to your model name
    )
    
    # Create dummy image
    dummy_image = np.zeros((540, 960, 3), dtype=np.uint8)
    # Add some visual content
    dummy_image[200:400, 400:560] = [100, 100, 100]  # Road-like rectangle
    
    # Predict
    print("\nSending test image to VLM...")
    trajectory, inf_time = predictor.predict(dummy_image)
    
    if trajectory:
        print(f"\nPredicted trajectory ({len(trajectory)} waypoints):")
        for i, wp in enumerate(trajectory):
            print(f"  {i+1}: pixel=({wp[0]:.0f}, {wp[1]:.0f}), distance={wp[2]:.0f}m")
    else:
        print("\nNo trajectory predicted (check if server is running)")
    
    print(f"\nInference time: {inf_time:.3f}s")
    
    # Convert to Waypoint2D
    if trajectory:
        waypoints_2d = trajectory_to_waypoints_2d(trajectory)
        print(f"\nConverted to {len(waypoints_2d)} Waypoint2D objects")
