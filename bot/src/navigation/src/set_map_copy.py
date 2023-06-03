#! /usr/bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import SetMap

def call_set_map_service():
    rospy.wait_for_service('/set_map')  # Wait for the set_map service to be available
    try:
        set_map = rospy.ServiceProxy('/set_map', SetMap)

        # Create the map
        map_msg = OccupancyGrid()
        pgm_file_path = '/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/map/map.pgm'
        # Convert PGM to OccupancyGrid
        map_msg = pgm_to_occupancy_grid(pgm_file_path)
        
        # Create the initial pose
        initial_pose_msg = PoseWithCovarianceStamped()
        # Fill in the necessary initial pose parameters
        # ...
        initial_pose_msg.header.frame_id = 'map'
        print(initial_pose_msg)
        #set_map.call(map_msg, initial_pose_msg)
        print("fatto")
        response = set_map(map_msg, initial_pose_msg)
        
        if response.success:
            rospy.loginfo("Map set successfully.")
        else:
            rospy.loginfo("Failed to set map.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def pgm_to_occupancy_grid(pgm_file):
    # Read PGM file
    with open(pgm_file, 'rb') as f:
        pgm_data = f.readlines()

    # Parse PGM header
    occ = pgm_data[2].decode().strip().split()
    print(occ)
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
    for line in pgm_data[5:]:
        values = line.split()
        for value in values:
            occupancy_grid.data.append(int(value.decode()))

    occupancy_grid.data = data
    print(occupancy_grid)
    return occupancy_grid

if __name__ == "__main__":
    rospy.init_node('pgm_to_occupancy_grid')
    # Convert PGM to OccupancyGrid

    call_set_map_service()