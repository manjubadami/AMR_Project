#! /usr/bin/env python
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion as EfQ 
from goal_publisher.msg import PointArray
import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import os
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from nav_msgs.msg import Path
#import sys
from geometry_msgs.msg import PoseStamped #, PoseWithCovarianceStamped


class global_path_dist():
    # Init module initialises all the member variables and subscribes to topics
    def __init__(self):
        rospy.Subscriber('/goals', PointArray, callback = self.__populate_goals)
        #rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback = self.__current_pos)
        rospy.Subscriber('odom', Odometry, callback = self.__populate_Odomvalues)
        self.goal = Point()
        self.current_pos = []
        self.limit = 2
        self.remaining_goals = []
        self.list_success = []
        self.next_goal = []
        self.cancelled_goals = []
        rospy.Subscriber('scan', LaserScan, callback = self.__populate_laser_scan)
        self.velocity = Twist()
        self.attempt = 0
        self.y = [100, 87, 73, 64, 48, 28, 18, 13, 10]
        self.x = [0.1, 0.16, 0.187, 0.24, 0.3, 0.5, 0.83, 0.9, 1]
        self.possible_edges = True
        self.manual_driving = True
        self.rewards_collected = []


    def __populate_laser_scan(self, msg):  # populates the laser scan values
        self.laser_scan = msg
        self.laser_fullrange = list(self.laser_scan.ranges[0:360])
        self.laser_fullrange[:] = [tpm if tpm != float("inf") else 50 for tpm in self.laser_fullrange]        
        self.laser_concernedrange = list(self.laser_scan.ranges[0:15] + self.laser_scan.ranges[345:360])
        self.laser_concernedrange[:] = [tpx if tpx != float("inf") else 50 for tpx in self.laser_concernedrange]  # 30 degrees front vision    
        self.laser_concernedrangereverse = list(self.laser_scan.ranges[165:195])
        self.laser_concernedrangereverse[:] = [tpx if tpx != float("inf") else 50 for tpx in self.laser_concernedrangereverse] # 30 degrees back vision
        self.laser_concernedrange_ext = list(self.laser_scan.ranges[0:30] + self.laser_scan.ranges[330:360])
        self.laser_concernedrange_ext[:] = [tpx if tpx != float("inf") else 50 for tpx in self.laser_concernedrange_ext]  # 30 degrees front vision
        #self.write_to_file(self.laser_fullrange, 1)
        
    def __populate_goals(self, msg): # populates the goals from the /goals topic
        self.goals = msg.goals

    def __current_pos(self, msg): # populates the pose of the robot using /amcl_pose topic
        self.currentpos = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        
    def __populate_Odomvalues(self,msg): # populates the pose of the robot using /Odom topic
        self.currentpos = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    # ros_publisher module initialises the publisher that is used to publish the velocity commands to the bot
    
    def ros_publisher(self):
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=25)
        self.rate = rospy.Rate(10)
        
        
    # comp_euclid module computes the euclidean distance between the two points as given in the input
        # inputs -- Member variables, coordinates
        # outputs -- returns the euclidean distance
        
    def comp_euclid(self,x1,y1,x2,y2):
        return (np.sqrt((x2-x1)**2 + (y2-y1)**2))
    
    
    # goal_pose function prepares the goal pose that is later sent to the move_base action server
    # inputs -- Member variables, pose
    # outputs -- goal_pose
    
    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[2][0]
        goal_pose.target_pose.pose.position.y = pose[2][1]
        goal_pose.target_pose.pose.position.z = 0
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0
        goal_pose.target_pose.pose.orientation.w = 1
        return goal_pose
        
    # get_angle_to_goal function calculates the angle between the current position of the bot and the goal_coordinates
    # inputs -- Member variables, x (goal_x_coordinate), y (goal_y_coordinate)
    # outputs -- ang (angle between current position and goal), sign (direction in which the bot is expected to rotate)
    
    def get_angle_to_goal(self,x,y):
        tempx = x - self.currentpos.x
        tempy = y - self.currentpos.y
        ang = math.atan2(tempy, tempx)
        sign = np.sign(ang - self.get_bot_current_angle(self.current_orientation))
        if abs(ang - self.get_bot_current_angle(self.current_orientation)) > np.pi :
            sign = sign * -1
        return ang, sign 
    
    
    # get_bot_current_angle_adapted function calculates the cuurent angle of the bot using it's current position, and the value is between (0 , 2*pi)
    # inputs -- Member variables, orientation (current orientation of the bot)
    # outputs -- angl (angle at which the bot is with respect to the world coordinates)
    
    def get_bot_current_angle_adapted(self, orientation):
        _, _, angl = EfQ([orientation.x, orientation.y, orientation.z, orientation.w])
        if np.sign(angl) == -1:
            angl = angl + (2* np.pi)
        return angl
    
    
    # get_bot_current_angle function calculates the cuurent angle of the bot using it's current position, and the value is between (0 , pi) and (-pi, 0)
    # inputs -- Member variables, orientation (current orientation of the bot)
    # outputs -- angl (angle at which the bot is with respect to the world coordinates)    

    def get_bot_current_angle(self, orientation):
        _, _, angl = EfQ([orientation.x, orientation.y, orientation.z, orientation.w])
        return angl
    
    
    # bot_move_angular module rotates the bot to the desired angle with desired speed by computing the angle to rotate using other member functions
        # inputs -- Member variables, goal coordinates (x,y), speed (speed of angular movement)
        # outputs -- None    

    def bot_move_angular(self,x,y,speed):
        while sum(ix < 0.12 for ix in self.laser_concernedrange) > 0:
            if sum(iy < 0.12 for iy in self.laser_concernedrangereverse) <= 0:
                self.velocity.linear.x = -0.1
                self.velocity_publisher.publish(self.velocity)
        self.velocity.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity)
        ang, sign = self.get_angle_to_goal(x,y)
        self.bot_move_angular_with_angle(ang,sign,speed)
        return ang
    
    
    # bot_move_angular_with_angle module rotates the bot to the desired angle as specified by the input "ang" in the direction specified by input "sign" with the speed specified by input "speed"
        # inputs -- Member variables, ang(angle to rotate in radians) ,sign(direction to rotate), speed (speed of angular movement)
        # outputs -- None    

    def bot_move_angular_with_angle(self,ang,sign,speed):
        while abs(ang - self.get_bot_current_angle(self.current_orientation)) > 0.2 : # rotate until bot has turned the required radians
            if all(x > 0.11 for x in self.laser_concernedrange) == False: # if obstacle is detected during turning, go in the reverse direction until bot is atleast 0.4 m away from obstacle
                while sum(ix < 0.11 for ix in self.laser_concernedrange) > 0:
                    if sum(iy < 0.11 for iy in self.laser_concernedrangereverse) <= 0:
                        self.velocity.linear.x = -0.1
                        self.velocity_publisher.publish(self.velocity)
                self.velocity.linear.x = 0.0
                self.velocity_publisher.publish(self.velocity)
                self.ObstacleDetected_avoid = True
            self.velocity.angular.z = speed * sign
            self.velocity.linear.x = 0.0
            self.velocity_publisher.publish(self.velocity)
        self.velocity.angular.z = 0.0
        self.velocity.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity)        
        
        
    # get_goal_laserIndex module returns the index(in turn, angle to the bot) at which the goal is with respect to the bot.
        # inputs -- Member variables, goal coordinates
        # outputs -- Index of the laser scan list at which the goal is wrt to the bot        
        
    def get_goal_laserIndex(self,x,y):
        angle, sign = self.get_angle_to_goal(x,y)
        if np.sign(angle) == -1:
            angle = angle + (2* np.pi)
        index =np.int(-1 * np.ceil(np.rad2deg(sign * (self.get_bot_current_angle_adapted(self.current_orientation) - angle))))
        if sign == -1 and np.sign(index) == -1:
            index = abs(index)
        elif sign == -1:
            index = 360 - index
        elif np.sign(index) == -1:
            index = 360 - abs(index)
        return index
    
    
    # get_angle_bw_two_points function calculates the angle between two input points with coordinates (x1,y1) , (x2,y2)
    # inputs -- Member variables, point_1 , point_2
    # outputs -- ang(angle between the two points)
    
    def get_angle_bw_two_points(self,goal_1,goal_2):
        tempx = goal_1[0] - goal_2[0]
        tempy = goal_1[1] - goal_2[1]
        ang = math.atan2(tempy, tempx)
        return ang    
    
    
     # return_num_ind_to_check module returns the number of indices to check for the second edge when stuck in a narrow path
     # it is done by interpolating from experiment derived values
        # inputs -- Member variables, distance (distance from bot to the first edge)
        # outputs -- Index of the laser scan list at which the goal is wrt to the bot   
    
    def return_num_ind_to_check(self,distance):
        return np.interp(distance, self.x, self.y)
        
            
    # write_to_file module is primarly used for debugging. This function writes the laser ranges into a text file
    
    def write_to_file(self,ranges,filename):
        filepath = os.path.join('/home/fatboy/temp', 'range' + str(filename) + '.txt')  # Creating a file to store msg.ranges values for debugging
        with open(filepath, 'w+') as f:
            for item in ranges:
                f.write("%s\n" % item)
            f.close()
    
    
    # get_global_plan_poses module makes a global plan and returns the poses. It is helpful in calculating the distance between bot and goal. (better than euclidean distance)
    # inputs -- Member variables, goals (current goal coordinates)
    # outputs -- x_coordinates, y_coordinates
    
    def get_global_plan_poses(self,goals):
        rospy.wait_for_service('move_base/NavfnROS/make_plan')
        global_path = rospy.ServiceProxy('move_base/NavfnROS/make_plan', GetPlan)
        temp = GetPlanRequest()
        pose_start = PoseStamped()
        pose_goal = PoseStamped()
        pose_start.header.seq = 0
        pose_start.header.frame_id = "map"
        pose_start.header.stamp = rospy.Time(0)
        pose_start.pose.position.x = self.currentpos.x
        pose_start.pose.position.y = self.currentpos.y
        temp.start = pose_start
        pose_goal.header.seq = 0
        pose_goal.header.frame_id = "map"
        pose_goal.header.stamp = rospy.Time(0)
        pose_goal.pose.position.x = goals[0]
        pose_goal.pose.position.y = goals[1]
        temp.goal = pose_goal
        temp.tolerance = 0.2
        result = global_path(temp)
        x_coordinates = []
        y_coordinates = []
        for value in range(len(result.plan.poses)):
            x_coordinates.append(result.plan.poses[value].pose.position.x)
            y_coordinates.append(result.plan.poses[value].pose.position.y)
        return x_coordinates, y_coordinates
    
    
    # get_angle_to_rot_from_desired module computes the angle the bot should be at inorder to turn the required number of radians as specified by the "req" input
        # inputs -- Member variables , req (number of radians to turn)
        # outputs -- ang_to_be (angle at which the bot is expected to be inorder to have turned the "req" number of radians)    
    
    def get_angle_to_rot_from_desired(self, req):
        current_angle = self.get_bot_current_angle(self.current_orientation)
        ang_to_be = current_angle + req
        if ang_to_be < -np.pi:
            ang_to_be = np.pi + (np.mod(ang_to_be , -np.pi))
        elif ang_to_be > np.pi:
            ang_to_be = -np.pi + (np.mod(ang_to_be, np.pi))
        return ang_to_be
    

    # bot_move_at_steady_speed module moves the bot at a steady speed of 0.05 m/s. This is particularly helpful when the bot needs to be moved through narrow spaces
        # inputs -- Member variables
        # outputs -- returns the parameter "ObstacleDetected" ("True" if an obstacle is detected while moving, "False" otherwise)

    def bot_move_at_steady_speed(self):
        ObstacleDetected = False
        if sum(tx < 0.15 for tx in self.laser_concernedrange_ext) > 0:  # if obstacle is detected within 1m of the viewing range set ObstacleDetected to "True"
            ObstacleDetected = True
        else:
            self.velocity.linear.x = 0.05
            self.velocity.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity)
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity)  # stop the bot
        return ObstacleDetected
 
    # rhombus_points module calculates the intermediate goals the bot should reach inorder to pass through the narrow corridor.
    # it calculates two goals which will align the bot perpendicular to the gap
    # inputs -- Member variables, edge_1 (coordinates of the first edge of the narrow space), edge_2 (coordinates of the second edge of the narrow space),
    #           height (indicates how far the intermediate goals needs to be placed from the narrow space)
    # output -- point_1 (intermediate goal 1), point_2 (intermediate goal_2)
    
    def rhombus_points(self, edge_1, edge_2, height):
        inc_x =  edge_1.x - edge_2.x
        inc_y =  edge_1.y - edge_2.y
        slope = math.atan2(inc_y, inc_x)
        mid_x = (edge_1.x + edge_2.x)/2
        mid_y = (edge_1.y + edge_2.y)/2
        point_1 = Point()
        point_2 = Point()
        point_1.x = mid_x + (height*np.sin(slope))
        point_1.y = mid_y - (height*np.cos(slope))
        point_2.x = mid_x - (height*np.sin(slope))
        point_2.y = mid_y + (height*np.cos(slope))
        return point_1, point_2
    
    
    # bot_move_linear module is the main module which helps the bot reach the intermediate goals inorder to pass through narrow corridor.
        # inputs -- Member variables, closest_goal( goal coordinates (x,y))
        # outputs -- ObstacleDetected (returns True if obstacle is detected, otherwise False)    
    
    def bot_move_linear(self,closest_goal):
        loopindex = 0
        ObstacleDetected = False
        euclidlist = [100] # create a list which has euclidean distances with the first value as 100
        euclidlist.append(self.comp_euclid(self.currentpos.x, self.currentpos.y, closest_goal.x, closest_goal.y))
        while (euclidlist[loopindex+1] - euclidlist[loopindex]) < 0.0005:
            if self.comp_euclid(self.currentpos.x, self.currentpos.y, closest_goal.x, closest_goal.y) <= 0.02:
                break
            if sum(tx < 0.12 for tx in self.laser_concernedrange_ext) > 0: # if bot detects an obstacle, break
                    ObstacleDetected = True
                    break
            if sum(tx < 0.5 for tx in self.laser_concernedrange_ext) > 0:
                self.velocity.linear.x = 0.05
            else:
                self.velocity.linear.x = 0.1
            self.velocity.angular.z = 0.0
            self.velocity_publisher.publish(self.velocity)
            euclidlist.append(self.comp_euclid(self.currentpos.x, self.currentpos.y, closest_goal.x, closest_goal.y))
            loopindex += 1
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_publisher.publish(self.velocity) 
        return ObstacleDetected
    
    
    # find_the_gap module tries to find the indices of laser scan at which the start and the end of the narrow path opening occurs.
    # inputs -- Member variables, goals (current goal coordinates), global_pose (coordinate of a point in the global plan), ranges (laser scan data)
    # outputs -- start_index (laser scan index of the first edge of the narrow space), end_index (laser scan index of the second edge of the narrow space), laser_direction (direction in which the narrow path occurs)
    
    def find_the_gap(self,goals, global_pose, ranges):
        goal_index = self.get_goal_laserIndex(goals[0],goals[1])
        angle_bw_goal_global_path = self.get_angle_bw_two_points(goals, global_pose)
        if np.sign(angle_bw_goal_global_path):
            laser_direction = -1
        else:
            laser_direction = 1
        check_index_1 = goal_index
        check_index_2 = check_index_1 + laser_direction
        while abs(ranges[check_index_1%360] - ranges[check_index_2%360]) < 0.5:
            check_index_1 = check_index_1 + laser_direction
            check_index_2 = check_index_1 + laser_direction
        start_index = check_index_1
        dist_to_edge_1 =  ranges[check_index_1]          
        check_indices = np.floor(self.return_num_ind_to_check(dist_to_edge_1))
        check_index_1 = check_index_2
        check_index_2 = check_index_1 + laser_direction
        possible_edges = []
        while check_indices != 0:
            if abs(ranges[check_index_1%360] - ranges[check_index_2%360]) > 0.5:
                possible_edges.append(check_index_2)
            check_index_1 = check_index_1 + laser_direction
            check_index_2 = check_index_1 + laser_direction
            check_indices -= 1
        if possible_edges != []:
            end_index = possible_edges[-1] % 360
        else:
            self.possible_edges = False
            end_index = 400
        return start_index, end_index, laser_direction
        
    
    # get_mid_point_gap module returns the midpoint and the angle to this point using the starting and ending laser index with respect to the narrow corridor
    # inputs -- Member variables, start_index (index the first edge of narrow corridor), end_index (index of the second edge of narrow corridor), laser_direction (direction in which the scan should be checked)
    # outputs -- midpoint (index of the midpoint), mid_angle (laser index of the midpoint)
    def get_mid_point_gap(self, start_index, end_index, laser_direction):
        midpoint = abs(start_index - end_index)
        if midpoint > 180:
            midpoint = (360 - midpoint) / 2
        else:
            midpoint = midpoint / 2
        mid_angle = (start_index + (laser_direction * midpoint)) % 360
        return midpoint, mid_angle


    # get_rotation_angle_gap module calculates the angle from the mid_angle as returned by the member function get_mid_point_gap
    # inputs -- Member variables, mid_angle ( mid angle of the narrow corridor)
    # outputs -- returns the angle in radians
    def get_rotation_angle_gap(self, mid_angle):
        if mid_angle <= 180:
            return np.deg2rad(mid_angle)
        else:
            return np.deg2rad(mid_angle - 360)


    # manual_driving_mode module helps the bot pass through narrow corridors by using several other member functions
    # inputs -- goal (goal coordinates), global_path_pose ( coordinates of one point in the global plan)
    # outputs -- obstacle (returns True if obstacle was encountered, False otherwise) , sets member variable (self.manual_driving to False if unsuccessful, True otherwise)
    
    def manual_mode_driving(self, goal, global_path_pose):
	rospy.loginfo("Seems like move_base has an issue. Switching to the manual driving mode")
        ranges = self.laser_fullrange
        start_index, end_index, laser_direction = self.find_the_gap(goal, global_path_pose, ranges)
        if self.possible_edges == True:
            if ranges[start_index] < 0.7 and ranges[end_index] < 0.7:
                _, mid_angle = self.get_mid_point_gap(start_index, end_index, laser_direction)
                angle = self.get_rotation_angle_gap(mid_angle)
                angle_to_be = self.get_angle_to_rot_from_desired(angle)
                sign = np.sign(angle_to_be - self.get_bot_current_angle(self.current_orientation))
                if abs(angle_to_be - self.get_bot_current_angle(self.current_orientation)) > np.pi :
                    sign = sign * -1 
                self.bot_move_angular_with_angle(angle_to_be,sign,0.3)
                start_index_angle = self.get_angle_to_rot_from_desired(self.get_rotation_angle_gap(start_index))
                end_index_angle = self.get_angle_to_rot_from_desired(self.get_rotation_angle_gap(end_index))
                r_start_index = ranges[start_index]
                r_end_index = ranges[end_index]
                edge_1 = Point()
                edge_2 = Point()
                edge_1.x = self.currentpos.x + r_start_index*np.cos(start_index_angle)
                edge_1.y = self.currentpos.y + r_start_index*np.sin(start_index_angle)
                edge_2.x = self.currentpos.x + r_end_index*np.cos(end_index_angle)
                edge_2.y = self.currentpos.y + r_end_index*np.sin(end_index_angle)  
                minigoal_1, minigoal_2 = self.rhombus_points(edge_1,edge_2,height=0.3)
                euclid_1 = self.comp_euclid(self.currentpos.x, self.currentpos.y, minigoal_1.x, minigoal_1.y)
                euclid_2 = self.comp_euclid(self.currentpos.x, self.currentpos.y, minigoal_2.x, minigoal_2.y)
                if euclid_1 <= euclid_2:
                    closest_goal = minigoal_1
                    second_point = minigoal_2
                else:
                    closest_goal = minigoal_2
                    second_point = minigoal_1
                self.bot_move_angular(closest_goal.x,closest_goal.y,0.3)     
                obstacle = self.bot_move_linear(closest_goal)
                self.bot_move_angular(second_point.x,second_point.y,0.3)
                obstacle = self.bot_move_linear(second_point)
            else:
                self.manual_driving = False
                obstacle = False 
        else:
            self.manual_driving = False
            obstacle = False
        return obstacle
    
    
    # serv_call module calculates the distances between the current position of the bot and all the goal coordinates
    # inputs -- Member variables, goals (list of all the goals)
    # outputs -- distances (list of distances from current position to each of the goals)
    
    def serv_call(self, goals):
        distances = []
        #from time import time
        #tic = time()
        for index in range(len(goals)):
            x_coordinates, y_coordinates = self.get_global_plan_poses(goals[index])
            if x_coordinates != []:
                distance = 0
                for i in range(len(x_coordinates)-1):
                    if i == 0:
                        distance += self.comp_euclid(self.currentpos.x,self.currentpos.y,x_coordinates[i],y_coordinates[i])
                    else:
                        distance += self.comp_euclid(x_coordinates[i],y_coordinates[i],x_coordinates[i+1],y_coordinates[i+1])
                distances.append(distance)
            else:
                distances.append(100)
        return distances
    
    
    # get_the_goal module returns the goal to be focussed on next, by using the sort_cost list which contains the cost of each goal.
    # inputs -- Member variables, sort_cost (list of cost to each goal from current position of the bot)    
    # outputs -- goal (goal coordinates), goal_full (tuple containing distance to goal, cost and goal coordinates)
    
    def get_the_goal(self,sort_cost):
        flag = 0
        for sc in sort_cost:
            if sc[0] < self.limit:
                goal = sc[2]
                goal_full = sc
                flag = 1
                break
            else:
                continue
        if flag == 0:
            sort_dist = sorted(sort_cost,key = lambda x: x[0])
            goal = sort_dist[0][2]
            goal_full = sort_dist[0]
        return goal,goal_full
    
    
    # sort_goals_cost module calculates the next goal using the distances from the serv_call function and goal coordinates.
    # inputs -- Member variables, distance (distances including the obstacles returned by serv_call member function), goal_coords (goal coordinates)
    # outputs -- None (updates the self.remaining_goals list)
    
    def sort_goals_cost(self,distance, goal_coords):
        total_dist = np.sum(distance)
        norm_dist = np.true_divide(distance,total_dist)
        rewards = zip(*goal_coords)[2]
        total_rewards = np.sum(rewards)
        norm_rewards = np.true_divide(rewards,total_rewards)
        cost = np.true_divide(norm_dist,norm_rewards)
        new_list = [list(x) for x in zip(distance, cost,goal_coords)]
        sort_cost = sorted(new_list,key = lambda x: x[1])
        goal_point,goal_full = self.get_the_goal(sort_cost)
        self.list_success.append(goal_full)
        self.next_goal = goal_full
        sort_cost.remove(goal_full)
        if sort_cost != []:
            self.remaining_goals = zip(*sort_cost)[2]
            self.remaining_goals = [i for i in self.remaining_goals]
        else:
             self.remaining_goals = []    
    
    
    # attempt_goals module is one of the main modules of this navigation script. Using various member variables, it will attempt reaching all the goals.
    # If it fails, it updates the self.cancelled_goals, which could be later used to reach all the cancelled goals
    # inputs -- Member variables 
    # outputs -- None (moves the bot to different goals and updates self.cancelled_goals)
    
    def attempt_goals(self):
        while len(self.remaining_goals) >= 1:
            distances = self.serv_call(self.remaining_goals)
            self.sort_goals_cost(distances,self.remaining_goals)
            rospy.loginfo("Attempting to reach goal : ({},{}) with {} reward points".format(self.next_goal[2][0],self.next_goal[2][1],self.next_goal[2][2]))
            x,y = self.get_global_plan_poses(self.next_goal[2])
            if x != []:
                self.bot_move_angular(x[(len(x)/4)],y[(len(x)/4)],speed=1.5)
            goal = self.goal_pose(self.next_goal)
            client.send_goal(goal)
            self.oldpos = self.currentpos
            self.count = 0
            state_result = client.get_state()
            import time
            dist1 = []
            while state_result < 2:
                state_result = client.get_state()
                dist = self.comp_euclid(self.oldpos.x, self.oldpos.y, self.currentpos.x, self.currentpos.y)
                self.oldpos = self.currentpos
                if dist < 0.05:
                    dist1.append(dist)
                if len(dist1) > 5:
                    client.cancel_goal()
                    rospy.logwarn("Bot immobile for quite some time. The goal ({},{}) with reward {} will be prempted and attempted at a later point in time".format(self.next_goal[2][0],self.next_goal[2][1],self.next_goal[2][2]))
                    break
                time.sleep(4)
            if state_result == 3:
                f = open("goals_reached_JiMaKi_final_challenge.txt","a+")
                f.writelines(str(self.next_goal[2]) + " \n")
                f.close()
                rospy.loginfo("Goal : ({},{}) reached with reward : {}".format(self.next_goal[2][0],self.next_goal[2][1], self.next_goal[2][2]))
                self.rewards_collected.append(self.next_goal[2][2])
                rospy.loginfo("Total reward points collected : {}".format(np.sum(self.rewards_collected)))
                continue
            else:              
                self.cancelled_goals.append(self.next_goal[2])
                
            for index in range(2):
                x,y = self.get_global_plan_poses(self.next_goal[2])
                if x != []:
                    self.bot_move_angular(x[(len(x)/4)],y[(len(x)/4)],speed=1.5)
                    if self.attempt >= 1:
                        obstacle = self.manual_mode_driving(self.next_goal[2], (x[min(10,len(x))],y[min(10,len(x))]))
                        if self.manual_driving == False:
                            self.manual_driving = True
                            self.possible_edges = True
                            break
                        elif obstacle:
                            break
                    else:
                        break


    # localise_robot module localises the robot in the map by rotating for the number of seconds as given by the input "rot_time"             
    # inputs -- Member variables, rot_time (time to which the bot must rotate)
    # outputs -- None (rotates the bot)

    def localise_robot(self, rot_time):
        self.velocity.angular.z = 3.0
        self.velocity.linear.x = 0.0
        from time import time
        timer = time()
        while time()-timer < rot_time:
            self.velocity_publisher.publish(self.velocity)
        self.velocity.angular.z = 0.0
        self.velocity.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity)
        return
                    
                    
    # head is the main function of this class which calls other member functions to perform bot movement to goals.
    # inputs -- Member variables
    # outputs -- None (prints on the console)
    def head(self):
        self.ros_publisher()
        rospy.loginfo("Localizing the turtlebot in the map")
        self.localise_robot(10)
        goals = []
        for index in range(len(self.goals)):
            goals.append((self.goals[index].x, self.goals[index].y, self.goals[index].reward ))
        self.remaining_goals = goals
        reached_goals = []
        f = open("goals_reached_JiMaKi_final_challenge.txt","a+")
        lines = f.readlines()
        if lines != []:
            for goals in self.remaining_goals:
                if (str(goals) + " \n") in lines:
                    reached_goals.append(goals)
            for goals in reached_goals:
                self.remaining_goals.remove(goals)
        f.close()
        while len(self.remaining_goals) >= 1:
            self.attempt_goals()
            self.attempt += 1
            self.remaining_goals = self.cancelled_goals
            self.cancelled_goals = []
        rospy.loginfo("Successfully reached all the goals. Reward points collected : {}".format(np.sum(self.rewards_collected)))
        return 
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('service_client')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        global_path_dist_inst = global_path_dist() # instantiate the class global_path_dist()
        time.sleep(5)
        global_path_dist_inst.head() # call the function serv_call
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

