import redis

r = redis.Redis(host='100.99.198.49', port=6379, db=0)

def health_check():
    try:
        r.ping()
        return True
    except redis.ConnectionError:
        return False

def stream_setup():
    r.flushall()
    r.xgroup_create('camera_stream', 'stream_consumers', id='0', mkstream=True)
    print("Redis stream 'camera_stream' initialized.")

def push_frame(frame_base64):
    r.xadd('camera_stream', {'frame': frame_base64})

def read_latest_frame():
    entries = r.xrevrange('camera_stream', count=1)
    if entries:
        _, data = entries[0]
        return data.get(b'frame', None)
    return None