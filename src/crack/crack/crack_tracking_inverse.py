# 목표 : 사람(RC car)이 있는 위치까지 이동했다는 전제하에 해당 위치에서 제일 가까운 출구까지 이동한다.(균열이 없는 출구에 한해서 -> 입구에 균열이 둘다 없을 수도 있고 하나가 있을 수도 있으므로)
# 그 과정에서 rc car가 로봇을 따라오도록 하며 계속해서 로봇은 뒤돌아보면서 RC카 와의 거리 값을 체크하면서 일정 거리를 유지하는지 체크를 한 후 -> 차가 잘 따라오고 있으면 원래의 경로로 이동을 하고 
# -> 만약 사라지는 경우 로봇이 이전의 경유지 좌표로 이동하고,
# -> 만약 보이는데 거리가 먼 경우 일정 거리에 들어올때까지 로봇이 제자리에서 기다리는 시나리오
# -> 필요 요소 :스테레오 카메라로 rc카와 로봇 간의 거리를 map 상의 좌표 값으로 변환되어야하며 (-> 변환 될 필요가 없음
# 				-> 사용자를 찾은 시점에서 유클리드 거리 값으로 균열이 없는 제일 가까운 좌표로 이동해야한다. -> 전역 변수  : dictionary 출구1 : , 출구2, -> flag 값을 주입

 #!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)



# 목표 : 사람(RC car)이 있는 위치까지 이동했다는 전제하에 해당 위치에서 제일 가까운 출구까지 이동한다.(균열이 없는 출구에 한해서 -> 입구에 균열이 둘다 없을 수도 있고 하나가 있을 수도 있으므로)
# 그 과정에서 rc car가 로봇을 따라오도록 하며 계속해서 로봇은 뒤돌아보면서 RC카 와의 거리 값을 체크하면서 일정 거리를 유지하는지 체크를 한 후 -> 차가 잘 따라오고 있으면 원래의 경로로 이동을 하고 
# -> 만약 사라지는 경우 로봇이 이전의 경유지 좌표로 이동하고,
# -> 만약 보이는데 거리가 먼 경우 일정 거리에 들어올때까지 로봇이 제자리에서 기다리는 시나리오
# -> 필요 요소 :스테레오 카메라로 rc카와 로봇 간의 거리를 map 상의 좌표 값으로 변환되어야하며 (-> 변환 될 필요가 없음
# 				-> 사용자를 찾은 시점에서 유클리드 거리 값으로 균열이 없는 제일 가까운 좌표로 이동해야한다. -> 전역 변수  : dictionary 출구1 : , 출구2, -> flag 값을 주입


#경유지 도착 시 바라보는 방향 설정
    # NORTH = 0 -> 이거 사용
    # NORTH_WEST = 45 
    # WEST = 90 -> 이거 사용
    # SOUTH_WEST = 135
    # SOUTH = 180 -> 이거 사용
    # SOUTH_EAST = 225
    # EAST = 270 -> 이거 사용 
    # NORTH_EAST = 315

import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from crack_msgs.msg import Obstacle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys
from action_msgs.msg import GoalStatus
import time
from geometry_msgs.msg import Twist
from time import sleep
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import select
import termios
import tty
import math
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseWithCovarianceStamped

class crack_tracking_inverse(Node): # robot7
    def __init__(self):
        super().__init__('crack_tracking_inverse')
        self.unsafe_exit = {0:True , 1:True} # true -> you can escape! / false -> you can't escape
        self.select = 0
        self.previous_pose = PoseWithCovarianceStamped()
        self.current_pose = PoseWithCovarianceStamped() 
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self.group = ReentrantCallbackGroup()
        self.person = Obstacle() # real-time object detection
        self.obstacle_sub = self.create_subscription(Obstacle, '/obstacle', 
                                                      self.obstacle_callback, 10) # obstacle sub -> real-time detect
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback,10)
        self.rotate_pub = self.create_publisher(Twist, '/robot7/cmd_vel', 10 ) # obstacle sub
        self.navigator = TurtleBot4Navigator(namespace='robot7') # nav2에게 좌표 값을 주기 위해 객체 생성 이객체 안에 함수들을 통해 경유지 전송
        self.timer = self.create_timer(2.0, self.follow_people) # person follow check ->  1hz
        self.person_detect = False # set True after exit circulation 
        self.section_list = {0 : [0.116, 4.941], # 4 exit
                        1 : [0.211, 4.522], # 5 exit
                        }
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self,x,y): # exit position input
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set goal position (update these values as needed)
        goal_msg.pose.pose.position.x = x  # Target X coordinate
        goal_msg.pose.pose.position.y = y  # Target Y coordinate
        goal_msg.pose.pose.position.z = 0.0  # Z is typically 0 for 2D navigation

        # Set goal orientation using the euler_to_quaternion method
        goal_yaw = 0.0  # Target orientation in radians rotation
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, goal_yaw)

        # Wait for the action server to be available
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal...')
        
        # Send the goal asynchronously and set up a callback for response
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback) # feedback robot processing
        self._send_goal_future.add_done_callback(self.goal_response_callback) # complete?

    def goal_response_callback(self, future): # GOAL response ok or no
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle  # Save the goal handle to use for cancellation
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    def goal_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.send_goal(0.0,0.0)
            self.person_detect =False #escape waypoint goal move -> signal detection 
            self.navigator.dock() # why? change 
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was canceled.") # this part
        else:
            self.get_logger().warn(f"Goal ended with status: {result.status}")
        self.goal_sent = False

    def feedback_callback(self, feedback_msg): # real time LOG msg 
        self.get_logger().info(f'Current position: {feedback_msg.feedback.current_pose.pose}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f'Goal cancellation in progress: {len(cancel_response.goals_canceling)} goal(s) canceling...')

            # Wait for the goal state to be fully canceled
            self.wait_for_cancellation()

        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def wait_for_cancellation(self):
        """ Polls the goal state until it reaches STATUS_CANCELED """
        if self._goal_handle is None:
            self.get_logger().info('No active goal to check for cancellation.')
            return

        while True:
            result_future = self._goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result is not None and result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Goal successfully canceled. Exiting program...')
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            else:
                self.get_logger().info('Waiting for goal to be fully canceled...')
                time.sleep(0.1)
    
    def obstacle_callback(self,msg):
            self.person = msg
            self.get_logger().info(f"detect! distance : {self.person.obstacle_distance} / class : {self.person.class_name}")
            
            # elif :
            # self.get_logger().info("service server ready!")
    def follow_people(self): # loop per 2 second 
        if self.person_detect == True: # following
            self.cancel_goal()
            angular_velocity = 3.14159 / 2  # 180 degrees = π radians, 2 seconds
            # Twist 메시지 생성
            twist = Twist()
            twist.angular.z = angular_velocity  # 시계방향 회전
            # 2초 동안 계속해서 회전
            self.rotate_pub.publish(twist)
            sleep(2)  # 2초 동안 대기 for detection delay
            twist.angular.z = 0
            self.get_logger().info("Publishing twist message for rotation")
            if (self.person.obstacle_distance > 1.0 and self.person.obstacle_distance < 3.0) and self.person.class_name =='car':
                self.send_goal(self.section_list[self.select][0],self.section_list[self.select][1]) # exit position input
            elif self.person.obstacle_distance > 3.0 and self.person.class_name =='car': 
                while self.person.obstacle_distance < 3.0:# 사람이 인지가 되도 3.0m 이내가 아니면 제자리에서 기다리고
                    self.get_logger().info("wait...until person near")
            elif self.person.class_name !='car': #  3초 per waypoint save
                #self.person initialize objectdetection value 
                     self.send_goal(self.previous_pose.position.x,self.previous_pose.position.y)
                     self.person_detect =False #escape waypoint goal move -> signal detection 
            self.previous_pose = self.current_pose # resave waypoint


def main():
    rclpy.init()

    node = crack_tracking_inverse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(node)
    # executor.spin()
    # executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
