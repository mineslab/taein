#!/usr/bin/python3

##======================##
## Written by Taein Kim ##
##======================##

## 할로윈데이에 로봇이 사탕을 건네주는 시나리오 ##
# 로봇이 문으로 다가가는 동작
# 사탕 바구니를 내려놓는 동작
# 문을 두드리는 동작
# 사탕 바구니 좌우로 흔드는 동작

import rospy
from hiwonder_servo_msgs.msg import *
from chassis_control.msg import *
import math


def stop_chassis():
    velocity = 0
    direction = 0
    angular = 0
    set_velocity_pub.publish(velocity, direction, angular)
    print('Chassis Stop.')

def connect_to_chassis_subscriber():
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

def translate_chassis(velocity, direction, angular, distance):
    '''
    평행이동 함수 -> velocity 속도와 direction 방향으로 distance 거리만큼 평행이동함.
    - velocity: 속도 (mm/s)
    - direction: 방향 (0~360 degree)
    - angular: 각속도 (rad/s)
    - distance: 거리 (mm)
    '''
    try:
        if velocity > 0:
            time = int(distance / velocity) # 단위: s
            # 한번 모터에 해당 velocity, direction, angular 데이터를 보냈으니, ROS가 아무것도 안하고 잠자고(sleep) 있을 때는 계속 그 값으로 모터가 동작함. 그래서 멈추지 않고 계속 움직이는 것임.
            rospy.loginfo('Translate for {} seconds.'.format(time))
            set_velocity_pub.publish(velocity, direction, angular)
            rospy.sleep(time)
        else:
            set_velocity_pub.publish(velocity, direction, angular)
    except Exception as e:
        print('[Error]:', e)

def rotate_chassis(velocity, direction, angular, degree):
    '''
    회전 함수 -> angular 각속도로 degree 각도만큼 회전함.
    - velocity: 속도 (mm/s)
    - direction: 방향 (0~360 degree)
    - angular: 각속도 (rad/s)
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

def rotate_servo(duration, id_pos_s):
    '''
    서보 모터를 회전시키는 함수.
    - duration : 서보 모터의 회전 시간 (범위 : 0 ms ~ 30000 ms)
    - id_pos_s: (id, position) 튜플
      - id : 서보 모터의 ID
      - position : 서보 모터의 위치 (범위 : 0 ~ 1000) (펄스 = 회전각?)
    '''
    id_pos_dur_list = list(map(lambda x : RawIdPosDur(x[0], x[1], duration), id_pos_s))
    msg = MultiRawIdPosDur(id_pos_dur_list = id_pos_dur_list)

    time = round(duration / 1000, 1) # 단위: s
    rospy.loginfo('Rotate for {} seconds.'.format(time))
    joints_pub.publish(msg)
    rospy.sleep(time)



if __name__ == '__main__':
    rospy.init_node('scenario1', anonymous = True)

    set_velocity_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size = 1)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size = 1)

    rospy.on_shutdown(stop_chassis)

    shortcuts_chassis = {
        'move_forward_50_cm': {
            'velocity': 120,
            'direction': 90, 
            'angular': 0, 
            'distance': 500
        },
        'stop_chassis': {
            'velocity': 0,
            'direction': 0, 
            'angular': 0, 
            'distance': 0
        }
    }

    shortcuts_servo = {
        'init': {
            'duration': 500, 
            'id_pos_s': ((1, 1000), (2, 500), (3, 260), (4, 706), (5, 212), (6, 500))
        },
        'put_down_candy': {
            'duration': 1000,
            'id_pos_s': ((2, 500), (3, 188), (4, 694), (5, 129), (6, 706))
        },
        'raise_head': {
            'duration': 1000,
            'id_pos_s': ((3, 365),)
        },
        'turn_left_head':{
            'duration': 1000,
            'id_pos_s': ((6, 706),)
        },
        'knock_door': {
            'duration': 500,
            'id_pos_s': ((4,588), (5, 94))
        },
        'open_gripper': {
            'duration': 1000,
            'id_pos_s': ((1, 400),)
        },
        'close_gripper': {
            'duration': 1000,
            'id_pos_s': ((1, 1000),)
        },
        'swing_left_gripper': {
            'duration': 500,
            'id_pos_s': ((2, 700),)
        },
        'swing_right_gripper': {
            'duration': 500,
            'id_pos_s': ((2, 300),)
        }
    }

    trick_or_treat = ['init', 
                      'move_forward_50_cm', 'stop_chassis',
                      'put_down_candy', 'open_gripper', 'raise_head', 'init',
                      'knock_door', 'init', 'knock_door', 'init', 'knock_door', 'init', 
                      'raise_head', 'turn_left_head', 'open_gripper', 'put_down_candy', 'close_gripper', 'init',
                      'swing_left_gripper', 'swing_right_gripper',
                      'swing_left_gripper', 'swing_right_gripper',
                      'swing_left_gripper', 'swing_right_gripper', 'init']

    # 섀시 subscriber와 연결.
    connect_to_chassis_subscriber()

    # Trick-or-Treat
    for pose in trick_or_treat:
        if pose == 'move_forward_50_cm' or pose == 'stop_chassis':
            action = shortcuts_chassis[pose]
            velocity = action['velocity']
            direction = action['direction']
            angular = action['angular']
            distance = action['distance']
            rospy.loginfo('Velocity: %s, Direction: %s, Angular: %s, Distance: %s', velocity, direction, angular, distance)
            translate_chassis(velocity, direction, angular, distance)
        else:
            action = shortcuts_servo[pose]
            duration = action['duration']
            id_pos_s = action['id_pos_s']
            rospy.loginfo('Duration: %s, Id_Pos: %s', duration, id_pos_s)
            rotate_servo(duration, id_pos_s)
        rospy.loginfo('Action: %s', pose)




