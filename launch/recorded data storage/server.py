import socket
import time

# Server setup
UDP_IP = "192.168.137.64"  # Replace with your server's IP address
UDP_PORT = 8000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Server started at {}:{}".format(UDP_IP, UDP_PORT))

try:
    while True:
        # Receive data from client
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        receive_time = time.time()

        pwm, sent_time = data.decode().split('|')
        sent_time = float(sent_time)
        elapsed_time = receive_time - sent_time

        print(f"Received PWM: {pwm}, Sent time: {sent_time}, Received time: {receive_time}, Elapsed time: {elapsed_time}s")

        # Log data in file
        with open('hardware_stress_test.txt', 'a') as file:
            file.write(f"Received PWM: {pwm}, Sent time: {sent_time}, Received time: {receive_time}, Elapsed time: {elapsed_time}s\n")

except KeyboardInterrupt:
    print("Server is shutting down.")

finally:
    sock.close()
