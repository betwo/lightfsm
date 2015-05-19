#!/usr/bin/env python
from twisted.test.test_process import GetArgumentVector
import roslib
import rospy
import smach
import smach_ros
from path_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import tf
import tf.transformations
import numpy as np
import cv2
import cv
import math

roslib.load_manifest('hlctrl')

class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'error'])

    def execute(self, userdata):
        rospy.loginfo("SpaceBot Cup 2015 - Team Attempto")
        rospy.loginfo("Initiating HighLevel Control")
        return 'success'


# define state Bar
class GoalSelector(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_selected', 'error'],
                             output_keys=['goal_pose', 'velocity'])

    def execute(self, userdata):
        rospy.loginfo('Exploring the environment')

        rospy.wait_for_service('/dynamic_map')
        try:
            client = rospy.ServiceProxy('/dynamic_map', GetMap)
            r = client()
            grid = r.map
            rospy.loginfo("got map of size %d" % grid.info.width )

            p = self.select_goal(grid)
            userdata.goal_pose = p
            userdata.velocity = 0.5
            rospy.loginfo("selected goal %d, %d" % (p.pose.position.x, p.pose.position.y) )

            return 'goal_selected'

        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s" % e)
            return 'error'

    def select_goal(self, grid):
        global g_listener

        try:
            t = g_listener.getLatestCommonTime("/base_link", "/map")
            (trans, rot) = g_listener.lookupTransform('/map', '/base_link', t)

            w = grid.info.width
            h = grid.info.height

            map = np.zeros((h,w,1), np.uint8)
            img = np.zeros((h,w,3), np.uint8)

            free = np.zeros((h,w,1), np.uint8)
            obstacles = np.zeros((h,w,1), np.uint8)

            for y in range(h):
                for x in range(w):
                    v = grid.data[y*w + x]
                    map[y][x] = v
                    obstacles[y][x] = 255
                    if v >= 0 and v < 60:
                        free[y][x] = 255
                    elif v > 60:
                        obstacles[y][x] = 0

            kernel = np.ones((5,5),np.uint8)
            free = cv2.dilate(free, kernel, iterations=1)
            kernel = np.ones((3,3),np.uint8)
            obstacles = cv2.erode(obstacles, kernel, iterations=1)

            free_distance = cv2.distanceTransform(free, cv.CV_DIST_L2, 5)
            obstacles_distance = cv2.distanceTransform(obstacles, cv.CV_DIST_L2, 3)


            for y in range(h):
                for x in range(w):
                    img[y][x] = map[y][x]

            res = grid.info.resolution
            rx = int((trans[0] - grid.info.origin.position.x) / res)
            ry = int((trans[1] - grid.info.origin.position.y) / res)

            min_dist_to_start = 2.0
            desired_frontier_dist = 1.0
            min_dist_to_obstacles = desired_frontier_dist


            Q = [(rx, ry)]
            visited = [[False for x in range(w)] for y in range(h)]
            dx = [-1, 0, 1, 0]
            dy = [0, -1, 0, 1]

            candidates = []

            while Q:
                (x,y) = Q.pop(0)

                frontier_dist = free_distance[y][x] * res
                obstacle_dist = obstacles_distance[y][x] * res
                start_dist = math.hypot(x-rx, y-ry) * res

                close_to_frontier = abs(frontier_dist - desired_frontier_dist) < 0.1
                far_from_obstacle = abs(obstacle_dist) > min_dist_to_obstacles
                far_from_start = start_dist > min_dist_to_start

                if close_to_frontier and far_from_obstacle and far_from_start:
                    img[y][x] = (0,0,255)
                    candidates.append((x,y))

                for i in range(len(dx)):
                    nx = x + dx[i]
                    ny = y + dy[i]

                    if nx < 0 or nx >= w or ny < 0 or ny >= h:
                        continue

                    if not visited[ny][nx]:
                        if free[ny][nx] > 0:
                            Q.append((nx, ny))
                            visited[ny][nx] = True

            pos = candidates[0] if candidates else (rx, ry)

            Q = [pos]
            visited = [[False for x in range(w)] for y in range(h)]

            lookat = pos

            while Q:
                (x,y) = Q.pop(0)

                for i in range(len(dx)):
                    nx = x + dx[i]
                    ny = y + dy[i]

                    if nx < 0 or nx >= w or ny < 0 or ny >= h:
                        continue

                    if not visited[ny][nx]:
                        if free[ny][nx] > 0:
                            Q.append((nx, ny))
                            visited[ny][nx] = True
                        elif obstacles[ny][nx] != 255:
                            lookat = (nx, ny)
                            break

            #angle = math.atan2(lookat[1] - pos[1], lookat[0] - pos[0]) + math.pi
            angle = math.atan2(pos[1] - ry, pos[0] - rx)
            rot = tf.transformations.quaternion_about_axis(angle, (0,0,1))



            rospy.loginfo("rx: %d, ry: %d" % (rx, ry))
            cv2.circle(img, (rx, ry), 1, (255,255,0), 3, cv2.CV_AA)
            cv2.circle(img, (pos[0], pos[1]), 1, (255,0,255), 3, cv2.CV_AA)

            img = cv2.resize(img, None, dst=img, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
            #cv2.imshow('map', img)
            #cv2.imshow('free', free)
            #cv2.imshow('obstacle', obstacles)
            #cv2.waitKey(1000)


            goal = geometry_msgs.msg.PoseStamped()
            goal.pose.position.x = pos[0] * res + grid.info.origin.position.x
            goal.pose.position.y = pos[1] * res + grid.info.origin.position.y
            goal.pose.orientation.x = rot[0]
            goal.pose.orientation.y = rot[1]
            goal.pose.orientation.z = rot[2]
            goal.pose.orientation.w = rot[3]

            return goal

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            raise NameError("cannot lookup own pose: %s" % e)


def fsm():
    global g_listener

    rospy.init_node('sbc15_highlevel_control', anonymous=True)

    g_listener = tf.TransformListener()

    rospy.sleep(1)

    goto = smach.StateMachine(['success', 'error'],
                              input_keys=['goal_pose','velocity'])
    with goto:
        smach.StateMachine.add('GOTO_GOAL',
                               smach_ros.SimpleActionState('navigate_to_goal',
                                                           path_msgs.msg.NavigateToGoalAction,
                                                           goal_slots=['goal_pose', 'velocity']),
                               transitions={'succeeded':'success',
                                            'preempted':'error',
                                            'aborted':'error'},
                               remapping={'goal_pose':'goal_pose',
                                          'velocity':'velocity'})

    sm = smach.StateMachine(outcomes=['error', 'success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Initialization(),
                               transitions={'success':'SELECT_GOAL',
                                            'error':'error'})
        smach.StateMachine.add('SELECT_GOAL', GoalSelector(),
                               transitions={'goal_selected':'GOTO',
                                            'error': 'error'},
                               remapping={'goal_pose':'sm_user_data_pose',
                                          'velocity': 'sm_user_data_vel'})
        smach.StateMachine.add('GOTO', goto,
                               transitions={'success':'SELECT_GOAL',
                                            'error':'error'},
                               remapping={'goal_pose':'sm_user_data_pose',
                                          'velocity': 'sm_user_data_vel'})

    # Execute SMACH plan
    return sm.execute()

if __name__ == '__main__':
    try:
        fsm()
    except rospy.ROSInterruptException:
        pass