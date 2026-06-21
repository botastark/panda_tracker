import socket

HOST = "0.0.0.0"  # Listen on all network interfaces
PORT = 5000
BUFFER_SIZE = 4096


def run_listener() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)

        print(f"Listening on port {PORT}...")
        connection, address = server_socket.accept()

        with connection:
            print(f"Connected by {address[0]}:{address[1]}")

            while True:
                data = connection.recv(BUFFER_SIZE)

                if not data:
                    print("Sender disconnected.")
                    break

                print(f"Received {len(data)} bytes:")
                print(data.decode("utf-8", errors="replace"))


if __name__ == "__main__":
    try:
        run_listener()
    except KeyboardInterrupt:
        print("\nListener stopped.")
    except OSError as error:
        print(f"Socket error: {error}")