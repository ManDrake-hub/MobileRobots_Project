from dynamic_reconfigure.client import Client

class Parameters:
    def __init__(self):
        self.set_amcl_params()
        self.set_move_base_params()
        self.set_fast()

    def set_amcl_params(self):
        # Set parameters for amcl node
        client = Client("amcl")
        params = {
            'min_particles': 1000,
            'force_update_after_initialpose': True,
            'force_update_after_set_map': True,
            'update_min_d': 0.1,
            'update_min_a': 0.1,
            'gui_publish_rate': 10.0
        }
        client.update_configuration(params)

    def set_move_base_params(self):
        # Set parameters for move_base node
        client = Client("move_base/DWAPlannerROS")
        params = {'xy_goal_tolerance': 0.5, 'yaw_goal_tolerance': 3.14}
        client.update_configuration(params)

        client = Client("move_base/global_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/global_costmap/inflation_layer")
        params = {'inflation_radius': 0.7}
        client.update_configuration(params)
        client = Client("move_base/local_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/local_costmap/inflation_layer")
        params = {'inflation_radius': 0.7}
        client.update_configuration(params)

        client = Client("move_base")
        params = {'max_planning_retries': -1}
        client.update_configuration(params)

        client = Client("move_base")
        params = {'conservative_reset_dist': 1, 'recovery_behavior_enabled': True}
        client.update_configuration(params)

    def set_slow(self):
        # Set slow movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
        params = {
            'max_vel_x': 0.15,
            'min_vel_x': -0.15,
            'max_vel_trans': 0.15,
            'min_vel_trans': 0.08,
            'max_vel_theta': 1.0,
            'min_vel_theta': 0.5,
            'acc_lim_x': 1.5,
            'acc_lim_theta': 2.5
        }
        client.update_configuration(params)

    def set_medium(self):
        # Set medium movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
        params = {
            'max_vel_x': 0.2,
            'min_vel_x': -0.2,
            'max_vel_trans': 0.2,
            'min_vel_trans': 0.14,
            'max_vel_theta': 1.5,
            'min_vel_theta': 0.75,
            'acc_lim_x': 2.0,
            'acc_lim_theta': 2.5
        }
        client.update_configuration(params)

    def set_fast(self):
        # Set fast movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
        params = {
            'max_vel_x': 0.26,
            'min_vel_x': -0.26,
            'max_vel_trans': 0.26,
            'min_vel_trans': 0.18,
            'max_vel_theta': 1.82,
            'min_vel_theta': 0.9,
            'acc_lim_x': 2.5,
            'acc_lim_theta': 3.2
        }
        client.update_configuration(params)