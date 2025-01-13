# taein

## ROS 폴더
chassis_test.py : 로봇의 섀시 제어 테스트 코드  
servo_test.py : 로봇의 서보 모터 제어 테스트 코드  
trick-or-treat.py : 로봇 제어 예시 코드  

### 코드 실행 전 준비 사항  
1. ROS Core 실행
    ```
    roscore
    ```
2. Chassis 노드 실행
    ```
    rosrun chassis_control chassis_control_node.py
    ```
3. Servo Motor 노드 실행
    ```
    roslaunch hiwonder_servo_controllers start.launch
    ```
4. 원하는 코드 실행 (e.g. trick-or-treat.py)
    ```
    rosrun [패키지 이름] trick-or-treat.py
    ```