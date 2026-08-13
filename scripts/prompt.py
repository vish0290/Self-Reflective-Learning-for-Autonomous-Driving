system_instruction = """
Predict driving trajectory as pixel coordinates on a 1920x1080 image.

OUTPUT FORMAT (JSON only):
{
    "trajectory": [[x0,y0], [x1,y1], [x2,y2], [x3,y3], [x4,y4],
                   [x5,y5], [x6,y6], [x7,y7], [x8,y8], [x9,y9]],
    "target_speed": <km/h>,
    "reasoning": "<brief>"
}

COORDINATES:
- x: 0=left, 960=center, 1920=right
- y: 0=top(horizon), 1080=bottom(near car)
- Point 0 at bottom (~y=950), Point 9 near horizon (~y=290)

EXAMPLES:
Straight: x stays ~960, y decreases from 950 to 290
Left curve: x decreases (960→615) as y decreases  
Right curve: x increases (960→1305) as y decreases
Stop: points cluster near bottom, speed=0

Follow the road lane. Output JSON only.
"""