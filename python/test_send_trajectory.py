import numpy as np
import serial
import time

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

zero_config = np.array([90., 90., 90., 90., 90., 70.])
def get_test_trajectory():
    '''
        Construct a trajectory applying a sin wave to the gripper.

    '''
    N = 100
    ts = np.linspace(0., 10., N)
    qs = np.repeat(zero_config[np.newaxis, :], N, axis=0)
    qs[:, 5] += np.sin(ts * 2.) * 30
    qs[:, 3] += np.sin(ts * 3.) * 30
    qs[:, 3] += np.sin(ts * 3.) * 30
    qs[:, 2] += np.sin(ts * 1.5) * 30
    qs[:, 1] += np.sin(ts * 1.5) * 30

    # Back into linear buffer.
    return pack_trajectory_to_buf(ts, qs)

if __name__ == "__main__":
    buf = get_test_trajectory()
    print(f"Buffer size {len(buf)}")

    while (1):
        try:
            ser = serial.Serial(port='COM11', baudrate=9600, timeout=.1)

            last_send_time = time.time() - 100
            while (1):
                if (ser.inWaiting() > 0):
                    # read the bytes and convert from binary array to ASCII
                    data_str = ser.read(ser.inWaiting()).decode('ascii') 
                    # print the incoming string without putting a new-line
                    # ('\n') automatically after every print()
                    print(data_str, end='') 
                time.sleep(0.1)

                if time.time() - last_send_time > 10.:
                    last_send_time = time.time()
                    ser.write(buf)

        except serial.serialutil.SerialException as e:
            print(f"Err {e}")
            time.sleep(0.05)
            continue
    