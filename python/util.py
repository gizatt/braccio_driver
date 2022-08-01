import numpy as np

def pack_trajectory_to_buf(ts, qs):
    '''
        ts: np array, N knot times.
        qs: Nx6 array of joint angles in degrees.
    '''
    N = len(ts)
    data = np.hstack([ts.reshape(-1, 1), qs]).flatten().astype(np.float32)
    checksum = data.view(np.uint).sum()

    data_buffer = bytearray()
    for k in range(4):
        data_buffer.append(255)
    data_buffer.append(N)
    data_buffer += data.tobytes()
    data_buffer += np.array([checksum], dtype=np.uint).tobytes()
    return data_buffer