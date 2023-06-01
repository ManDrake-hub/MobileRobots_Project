import numpy as np
import matplotlib.pyplot as plt

def read_pgm(file_path):
    with open(file_path, 'rb') as f:
        header = f.readline().decode().strip()
        while header.startswith('#'.encode()):
            header = f.readline().decode().strip()

        width, height = 0, 0
        max_val = 255

        if header == 'P2':
            width, height = map(int, f.readline().decode().split())
            max_val = int(f.readline().decode().strip())
        elif header == 'P5':
            dimensions = f.readline().decode().split()
            width, height = int(dimensions[0]), int(dimensions[1])
            max_val = int(f.readline().decode().strip())
        else:
            raise ValueError('Invalid PGM format')

        image_data = np.fromfile(f, dtype=np.uint8, count=width*height).reshape((height, width))
        return image_data


# Convert PGM to occupancy grid
def pgm_to_occupancy_grid(pgm_data, threshold):
    occupancy_grid = (pgm_data > threshold).astype(int)
    return occupancy_grid

# Example usage
pgm_file_path = '/home/francesca/Scaricati/MobileRobots_Project/bot/src/map2gazebo/map/map.pgm'
threshold = 128

pgm_data = read_pgm(pgm_file_path)
occupancy_grid = pgm_to_occupancy_grid(pgm_data, threshold)
# Visualization (optional)
plt.imshow(occupancy_grid, cmap='gray')
plt.show()
