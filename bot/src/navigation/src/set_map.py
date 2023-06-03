#! /usr/bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import SetMap

def pgm_to_occupancy_grid(pgm_file):
    # Read PGM file
    with open(pgm_file, 'rb') as f:
        pgm_data = f.readlines()

    # Parse PGM header
    occ = pgm_data[2].decode().strip().split()
   
    width, height = map(int, occ)

    # Create OccupancyGrid message
    occupancy_grid = OccupancyGrid()

    # Set metadata
    metadata = MapMetaData()
    metadata.width = width
    metadata.height = height
    metadata.resolution = 1.0  # Adjust as needed
    occupancy_grid.info = metadata

    # Set data
    data = []
    #print(pgm_data[4:])
    for line in pgm_data[4:]:
        print(f"lunghezza {len(line)}")
        values = line.split()
        print(f"lunghezza {len(values)}")
        for value in values:
            print("sto qua")
            try:
                value.decode()
                data.append(int(value.decode()))
            except Exception as e:
                # TODO: non va la decode - Giovanni
                print(e)
            
            
    occupancy_grid.data = data
    print(occupancy_grid)
    return occupancy_grid

if __name__ == "__main__":
    rospy.init_node('pgm_to_occupancy_grid')
    try:
        map_pub= rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        pgm_file_path = '/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/map/map.pgm'
        occu_map = pgm_to_occupancy_grid(pgm_file_path)
        map_pub.publish(occu_map)
        print("ho pubblicato")
    except:
        pass
    rospy.spin()