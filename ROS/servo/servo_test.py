#!/usr/bin/python3

##======================##
## Written by Taein Kim ##
##======================##

import rospy
from hiwonder_servo_msgs.msg import *



def rotate(duration, id_pos_s):
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
    rospy.init_node('servo_test', anonymous = True)

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size = 1)
    
    shortcuts = {
        'rotate': {
            'turn_right': {
                'duration': 1000,
                'id_pos_s': ((6, 106), (1, 500))
            },
            'turn_left': {
                'duration': 1000,
                'id_pos_s': ((6, 1000), (1, 500))
            }
        }
    }

    action1 = shortcuts['rotate']['turn_left']
    duration1 = action1['duration']
    id_pos_s1 = action1['id_pos_s']

    action2 = shortcuts['rotate']['turn_right']
    duration2 = action2['duration']
    id_pos_s2 = action2['id_pos_s']

    rospy.loginfo("Let's rotate servo!")

    rotate(duration1, id_pos_s1)
    rotate(duration2, id_pos_s2)


