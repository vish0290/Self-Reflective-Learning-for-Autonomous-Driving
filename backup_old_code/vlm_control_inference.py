#!/usr/bin/env python3
"""
VLM Control Command Inference Module

Communicates with a VLM server to predict direct control commands (steer, throttle, brake)
from camera images and vehicle state.

Usage:
    from vlm_control_inference import VLMControlPredictor

    predictor = VLMControlPredictor(base_url="http://localhost:1234/v1")
    control_dict = predictor.predict(camera_image, speed_kmh=30.0, navigation="Go straight")
    # Returns: {"steer": 0.0, "throttle": 0.5, "brake": 0.0}
"""

import json
import re
import base64
import numpy as np
from PIL import Image
from typing import Dict, Tuple, Optional
import time
from io import BytesIO


class VLMControlPredictor:
    """
    VLM-based control command prediction using OpenAI-compatible API.
    """

    SYSTEM_INSTRUCTION = """You are an autonomous driving assistant that predicts vehicle control actions from visual input.
Given:
- Front camera view of the road
- Current vehicle speed (km/h)
- Navigation command (Turn left/Turn right/Go straight/Follow lane)

Output the control commands in this exact format:
steer: <value>, throttle: <value>, brake: <value>
"""

    def __init__(self,
                 base_url: str = "http://localhost:1234/v1",
                 api_key: str = "not-needed",
                 model: str = "control-model",
                 lora_adapter: str = None):
        """
        Initialize VLM control predictor with OpenAI-compatible API.

        Args:
            base_url: API endpoint
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
            print(f"VLM Control client initialized: {self.base_url}")
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

    def predict(self,
                image: np.ndarray,
                speed_kmh: float,
                navigation: str,
                temperature: float = 0.1) -> Tuple[Optional[Dict[str, float]], float]:
        """
        Predict control commands from camera image and vehicle state.

        Args:
            image: RGB image as numpy array (H, W, 3)
            speed_kmh: Current vehicle speed in km/h
            navigation: Navigation command (e.g., "Turn left", "Turn right", "Go straight", "Follow lane")
            temperature: Sampling temperature (lower = more deterministic)

        Returns:
            (control_dict, inference_time) where control_dict is {"steer": float, "throttle": float, "brake": float}
            Returns (None, inference_time) on failure
        """
        if self.client is None:
            return self._mock_predict(navigation)

        # Convert image to base64
        image_b64 = self._image_to_base64(image)

        # Format user prompt with context
        user_text = f"""Current Status:
  - Speed: {speed_kmh:.1f} km/h
  - Navigation: {navigation}

"""

        # Build messages with system instruction
        messages = [
            {
                "role": "system",
                "content": [
                    {
                        "type": "text",
                        "text": self.SYSTEM_INSTRUCTION
                    }
                ]
            },
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
                        "text": user_text
                    }
                ]
            }
        ]

        # Call API
        start_time = time.time()
        try:
            # Build API call parameters
            # In vLLM OpenAI API, LoRA adapters are specified using the "model" parameter
            # The server is started with: --lora-modules {adapter_name}={adapter_path}
            # Then you use model={adapter_name} in the request
            model_name = self.lora_adapter if self.lora_adapter else self.model

            api_params = {
                "model": model_name,
                "messages": messages,
                "max_tokens": 250,
                # "temperature": temperature,
            }

            response = self.client.chat.completions.create(**api_params)
            inference_time = time.time() - start_time

            # Extract response text
            response_text = response.choices[0].message.content

            # Parse control commands
            control_dict = self._parse_control_commands(response_text)

            return control_dict, inference_time

        except Exception as e:
            print(f"VLM API error: {e}")
            return None, time.time() - start_time

    def _parse_control_commands(self, response: str) -> Optional[Dict[str, float]]:
        """
        Parse control commands from model response.

        Expected format: "steer: 0.0, throttle: 0.5, brake: 0.0"
        or variants like "steer:0.0,throttle:0.5,brake:0.0"
        """
        # Try to extract using regex patterns
        steer_match = re.search(r'steer\s*:\s*([-\d.]+)', response, re.IGNORECASE)
        throttle_match = re.search(r'throttle\s*:\s*([\d.]+)', response, re.IGNORECASE)
        brake_match = re.search(r'brake\s*:\s*([\d.]+)', response, re.IGNORECASE)

        if steer_match and throttle_match and brake_match:
            try:
                steer = float(steer_match.group(1))
                throttle = float(throttle_match.group(1))
                brake = float(brake_match.group(1))

                # Clamp values to valid ranges
                steer = max(-1.0, min(1.0, steer))
                throttle = max(0.0, min(1.0, throttle))
                brake = max(0.0, min(1.0, brake))

                return {
                    "steer": steer,
                    "throttle": throttle,
                    "brake": brake
                }
            except ValueError:
                pass

        return None

    def _mock_predict(self, navigation: str) -> Tuple[Dict[str, float], float]:
        """Return mock control commands for testing without model."""
        # Simple rule-based mock based on navigation
        navigation_lower = navigation.lower()

        if "left" in navigation_lower:
            control = {"steer": -0.3, "throttle": 0.4, "brake": 0.0}
        elif "right" in navigation_lower:
            control = {"steer": 0.3, "throttle": 0.4, "brake": 0.0}
        elif "straight" in navigation_lower or "lane" in navigation_lower:
            control = {"steer": 0.0, "throttle": 0.5, "brake": 0.0}
        else:
            control = {"steer": 0.0, "throttle": 0.4, "brake": 0.0}

        return control, 0.05


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    print("Testing VLM Control Inference module...")
    print("=" * 70)

    # Test with mock predictor (no server)
    print("\n1. Testing with MOCK predictor (no server required)...")
    predictor = VLMControlPredictor(base_url="http://fake-url:1234/v1")
    predictor.client = None  # Force mock mode

    # Create dummy image
    dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
    dummy_image[200:400, 200:440] = [100, 100, 100]  # Road-like area

    test_scenarios = [
        {"speed": 30.0, "navigation": "Go straight"},
        {"speed": 25.0, "navigation": "Turn left"},
        {"speed": 20.0, "navigation": "Turn right"},
        {"speed": 35.0, "navigation": "Follow lane"},
    ]

    for scenario in test_scenarios:
        control, inf_time = predictor.predict(
            dummy_image,
            speed_kmh=scenario["speed"],
            navigation=scenario["navigation"]
        )

        print(f"\nScenario: {scenario['navigation']} at {scenario['speed']:.0f} km/h")
        if control:
            print(f"  Steer:    {control['steer']:+.3f}")
            print(f"  Throttle:  {control['throttle']:.3f}")
            print(f"  Brake:     {control['brake']:.3f}")
            print(f"  Time:      {inf_time:.3f}s")
        else:
            print("  Failed to get control commands")

    print("\n" + "=" * 70)
    print("Test complete! To test with real VLM server:")
    print("  predictor = VLMControlPredictor(base_url='http://your-server:port/v1')")
    print("=" * 70)
