import rospy
import actionlib

from tiago_iaslab_simulation.msg import IRMoveAction, IRMoveFeedback, IRMoveResult
from move_base_msgs import msg as move_base_msgs


class ActionServer:
    def __init__(self):
        self.a_server = actionlib.SimpleActionServer("/ir_pose", IRMoveAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        feedback = IRMoveFeedback()
        result = IRMoveResult()
        rate = rospy.Rate(1)

        def feedback_callback(feedback):
            print('[Feedback] Going to Goal Pose...')

        client = actionlib.SimpleActionClient('/move_base', move_base_msgs.MoveBaseAction)
        client.wait_for_server()

        x, y, Rz = tuple(goal.position)
        # creates a goal to send to the action server
        goal = move_base_msgs.MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = Rz
        goal.target_pose.pose.orientation.w = 0.66

        # client.send_goal(goal, feedback_cb=feedback_callback)
        # client.wait_for_result()
        print('GOAL:', goal)

        print('[Result] State: %d' % (client.get_state()))
        result.status = client.get_state()
        feedback.status = client.get_state()
        self.a_server.publish_feedback(feedback)
        self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("ir_pose")
    s = ActionServer()
    rospy.spin()
