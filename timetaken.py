import numpy as np
from PIL import Image
import time

# Fake image data (1920x1080x4)
data = np.random.randint(0, 255, (1080, 1920, 4), dtype=np.uint8)

# Test NPY
start = time.time()
for i in range(240):
    np.save(f'/tmp/test_{i}.npy', data)
print(f"NPY: {time.time() - start:.2f}s for 240 images")

# Test PNG
start = time.time()
for i in range(240):
    img = Image.fromarray(data[:,:,:3])
    img.save(f'/tmp/test_{i}.png')
print(f"PNG: {time.time() - start:.2f}s for 240 images")