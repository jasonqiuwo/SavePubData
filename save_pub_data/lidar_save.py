import socket
import struct
import numpy as np
import csv

def connect_to_lidar(ip='192.168.0.1', port=2112):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    return s

def get_scan_data(sock):
    sock.send(b'\x02sEN LMDscandata 1\x03')
    data = sock.recv(8192)
    return data

def parse_data(data):
    data_str = data.decode('ascii')
    fields = data_str.split(' ')
    distances = fields[26:]
    distances = [int(d, 16) for d in distances]
    return distances

def save_to_csv(data, filename='lidar_data.csv'):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'z'])  # header
        for point in data:
            writer.writerow(point)

def polar_to_cartesian(ranges, angle_increment, angle_min):
    points = []
    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_increment
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        points.append((x, y, 0.0))  # Assuming 2D LiDAR, z=0
    return points

def main():
    ip = '192.168.0.1'  # Change this
    port = 2111
    angle_min = -np.pi / 2  
    angle_max = np.pi / 2
    angle_increment = (angle_max - angle_min) / 1080

    sock = connect_to_lidar(ip, port)

    try:
        for _ in range(15):
            raw_data = get_scan_data(sock)
            ranges = parse_data(raw_data)
            cartesian_points = polar_to_cartesian(ranges, angle_increment, angle_min)
            save_to_csv(cartesian_points, 'lidar_data.csv')

    finally:
        sock.close()

if __name__ == '__main__':
    main()
