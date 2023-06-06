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
    rospy.init_node("set map")
    call_set_map_service()
    print("ho pubblicato")
    rospy.spin()