import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## @brief Send a goal to the move_base action server
#  @param goal_pose PoseStamped The desired robot's goal pose
#  @return int succeeded or failed
def send_goal(goal_pose):
    """Send a goal to move_base

    Args:
    goal_pose (PoseStamped): The desired robot's goal pose.

    Returns:
    int: succeeded or failed
    """
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_state()

if __name__ == "__main__":
    rospy.init_node('send_goals')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # define  goal positions and orientations
    ## @brief List of goal poses.
    #  @details each entry contains the x, y, z, and orientation of a goal pose.
    goals = [
        {
            'x': 1.5,
            'y': -2.0,
            'z': 0.0,
            'w': 1.0
        },
        {
            'x': 10.0,
            'y': -2.0,
            'z': 0.0,
            'w': 0.5
        }
    ]

    for i, goal_dict in enumerate(goals):
        # iterate through each goal and send to move_base
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_dict['x']
        goal_pose.pose.position.y = goal_dict['y']
        goal_pose.pose.orientation.z = goal_dict['z']
        goal_pose.pose.orientation.w = goal_dict['w']

        result = send_goal(goal_pose)
        if result == 3:  # If state is SUCCEEDED
            rospy.loginfo("Goal {0} reached!".format(i+1))
        else:
            rospy.logwarn("Failed to reach goal {0}!".format(i+1))
