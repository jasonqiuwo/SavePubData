import csv

def save_lidar_data_to_csv(data, filename='lidar_data.csv'):
    # 'data' should be a list of tuples or lists containing x, y, z coordinates
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'z'])  # header
        writer.writerows(data)

# Example data
lidar_data = [(0.5, 1.2, 0.0), (1.0, 1.5, 0.0), (1.5, 1.8, 0.0)]
save_lidar_data_to_csv(lidar_data)
