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
    metadata.resolution = 0.05  # Adjust as needed
    metadata.origin.position.x = -32.507755
    metadata.origin.position.y = -27.073547
    occupancy_grid.info = metadata

    occupancy_grid.data = [x-128 for x in list(pgm_data[4])]
    print(occupancy_grid.info)
    return occupancy_grid

if __name__ == "__main__":
    rospy.init_node('pgm_to_occupancy_grid')
    map_pub= rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rospy.sleep(0.5)
    pgm_file_path = '/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/map/map.pgm'
    occu_map = pgm_to_occupancy_grid(pgm_file_path)
    map_pub.publish(occu_map)
    print("ho pubblicato")
    rospy.spin()