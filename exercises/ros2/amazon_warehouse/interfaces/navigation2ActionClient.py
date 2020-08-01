#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Shyngyskhan Abilkassov <s.abilkassov@gmail.com>
#

# from tf.transformations import quaternion_from_euler

from nav2_msgs.action import NavigateToPose
import geometry_msgs.msg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# import rospy
from std_srvs.srv import Empty

## Use this if after moving pallet, there are obstacles left on costmaps, which prevent proper movement after moving and dropping pallet
# def clearCostmaps():
#     rclpy.wait_for_service('/move_base/clear_costmaps')
#     clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

#     try:
#         print("Clearing costmap")
#         clear_costmaps()
#     except rospy.ServiceException as exc:
#         print("Service did not process request: " + str(exc))


class NavigateToPoseActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self._action_client  = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.node = Node
        
 

    def sendGoalToClient(self, posX, posY, yaw = 0):

        self.goal = NavigateToPose.Goal()

        self.goal.pose.header.frame_id = "map"
        # self.goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        self.goal.pose.pose.position.x = float(posX)
        self.goal.pose.pose.position.y = float(posY)

        print("Got Goal!")
        print(self.goal)
        # orientation_q = quaternion_from_euler(0, 0, yaw)

        self.goal.pose.pose.orientation.x = 0.0
        self.goal.pose.pose.orientation.y = 0.0
        self.goal.pose.pose.orientation.z = 0.0
        self.goal.pose.pose.orientation.w = 1.0


        # self.goal.target_pose.pose.orientation.x = orientation_q[0]
        # self.goal.target_pose.pose.orientation.y = orientation_q[1]
        # self.goal.target_pose.pose.orientation.z = orientation_q[2]
        # self.goal.target_pose.pose.orientation.w = orientation_q[3]


        self._action_client.wait_for_server()

        self._action_client.send_goal_async(self.goal)
        # self.data = [0, 0]
        # self.goal = None
        
    # def getResultFromClient(self):
    #     if (self.goal):
    #         return self.client.get_result()
    #     else:
    #         return None