import socket, struct
import numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 6501))   # listen on all interfaces
print("Matrix receiver listening on port 6501...")

while True:
    data, addr = sock.recvfrom(1024)
    if len(data) == 128:   # 16 doubles = correct format (4x4 transform)
        vals = struct.unpack("<16d", data)
        T = np.array(vals).reshape(4, 4)
        print("4x4 matrix received (128 bytes OK):")
        print(np.array2string(T, precision=4, suppress_small=True))
        print("-" * 40)
    else:
        print(f"unexpected size: {len(data)} bytes (expected 128)")
