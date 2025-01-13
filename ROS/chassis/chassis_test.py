#!/usr/bin/python3

##======================##
## Written by Taein Kim ##
##======================##

import rospy
from chassis_control.msg import *
import math


def stop():
    velocity = 0
    direction = 0
    angular = 0
    set_velocity_pub.publish(velocity, direction, angular)
    print('Chassis Stop.')

def connect_to_subscriber():
    '''
    publisher가 첫 번째 메시지를 보내고 난 뒤 subscriber가 바퀴 모터를 움직일 준비를 할 때까지 걸리는 시간(딜레이)이 대략 3초 정도임.
    내가 명령하는 대로 모터가 곧장 움직일 수 있도록 처음에 이 함수를 먼저 실행하기.
    '''
    velocity = 0
    direction = 0
    angular = 0
    rospy.loginfo('Connect to a chassis control node.')
    set_velocity_pub.publish(velocity, direction, angular) # 아무 값이나 publish하고,
    rospy.sleep(3) # subscriber가 모터를 움직일 준비를 마칠 때까지 3초간 대기.

def translate(velocity, direction, angular, distance):
    '''
    평행이동 함수 -> velocity 속도와 direction 방향으로 distance 거리만큼 평행이동함.
    - velocity: 속도 (mm/s)
    - direction: 방향 (0~360 degree)
    - angular: 각속도 (rad/s)
    - distance: 거리 (mm)
    '''
    try:
        time = int(distance / velocity) # 단위: s
        
        # 한번 모터에 해당 velocity, direction, angular 데이터를 보냈으니, ROS가 아무것도 안하고 잠자고(sleep) 있을 때는 계속 그 값으로 모터가 동작함. 그래서 멈추지 않고 계속 움직이는 것임.
        rospy.loginfo('Translate for {} seconds.'.format(time))
        set_velocity_pub.publish(velocity, direction, angular)
        rospy.sleep(time)
    except Exception as e:
        print('[Error]:', e)

def rotate(velocity, direction, angular, degree):
    '''
    회전 함수 -> angular 각속도로 degree 각도만큼 회전함.
    - velocity: 속도 (mm/s)
    - direction: 방향 (0~360 degree)
    - angular: 각속도 (rad/s) (각속도 부호에 따라 회전 방향이 결정됨.)
    - degree : 회전 각도 (degree)
    '''
    try:
        rad_per_deg = math.pi / 180
        rad = degree * rad_per_deg
        time = round(rad / abs(angular) - 1.5, 1) # 단위: s. 90도일 때 딱 시간이 맞음. 근데 다른 각도로 하면 안맞음.

        # 한번 모터에 해당 velocity, direction, angular 데이터를 보냈으니, ROS가 아무것도 안하고 잠자고(sleep) 있을 때는 계속 그 값으로 모터가 동작함. 그래서 멈추지 않고 계속 움직이는 것임.
        rospy.loginfo('Rotate for {} seconds.'.format(time))
        set_velocity_pub.publish(velocity, direction, angular)
        rospy.sleep(time)
    except Exception as e:
        print('[Error]:', e)



if __name__ == '__main__':
    rospy.init_node('chassis_test', anonymous = True)

    set_velocity_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size = 1)
    rospy.on_shutdown(stop) # main 함수가 끝날 때 stop 함수가 실행됨.
    
    shortcuts = {
        'translate': {
            'forward_10_cm': {'velocity': 30, 'direction': 90, 'angular': 0, 'distance': 100},
            'forward_20_cm': {'velocity': 30, 'direction': 90, 'angular': 0, 'distance': 200},
            'backward_10_cm': {'velocity': 30, 'direction': 270, 'angular': 0, 'distance': 100},
            'left_10_cm': {'velocity': 30, 'direction': 180, 'angular': 0, 'distance': 100},
            'right_10_cm': {'velocity': 30, 'direction': 360, 'angular': 0, 'distance': 100}
        },
        'rotate': {
            'left_90_degree': {'velocity': 0, 'direction': 0, 'angular': 0.3, 'degree': 90},
            'right_90_degree': {'velocity': 0, 'direction': 0, 'angular': -0.3, 'degree': 90},
        }
    }

    # subscriber와 연결.
    connect_to_subscriber()

    # 앞으로 10cm 전진.
    action = shortcuts['translate']['forward_10_cm']
    velocity = action['velocity']
    direction = action['direction']
    angular = action['angular']
    distance = action['distance']
    rospy.loginfo('Go 10 cm forward.')
    translate(velocity, direction, angular, distance)

    # 왼쪽으로 90도 회전.
    action = shortcuts['rotate']['left_90_degree']
    velocity = action['velocity']
    direction = action['direction']
    angular = action['angular']
    degree = action['degree']
    rospy.loginfo('Turn 90 degrees to the left.')
    rotate(velocity, direction, angular, degree)

    # 앞으로 20cm 전진.
    action = shortcuts['translate']['forward_20_cm']
    velocity = action['velocity']
    direction = action['direction']
    angular = action['angular']
    distance = action['distance']
    rospy.loginfo('Go 20 cm forward.')
    translate(velocity, direction, angular, distance)


    
    
    
