<<<<<<< HEAD
# 하드웨어

# 할일
- 아두이노 R4 WIFI, 차체, 모터, IMU센서 결합 (일부 완 : IMU센서를 위해 점퍼 케이블 필요)
- 주행 테스트 코드 구현
- ROS2에서 발생한 실시간 데이터들을 MQTT를 통해 아두이노에서 받기
- MQTT에서 메세지를 파싱
- 각속도와 선속도를 데이터를 받게 되는데 이 데이터들 어떻게 활용해서 똑같은 움직임을 구현할지 고민해야함
- 주행 코드 구현현

## 03/24 (월) : 차체 제작 및 센서 부착
- 자체를 메뉴엘대로 제작, IMU센서 결합을 위해 점퍼 케이블 2개 더 필요 또한 와이어 리스를 위해 배터리 6개 필요

## 03/24 (화) : IMU 센서 결합 및 직진 코드 작성
- 직진 코드 작성 및 IMU 센서 결합

## 03/25 (수)
- 터틀봇의 각속도, 선속도를 활용하여 왼쪽과 오른쪽 바퀴의 속도를 구하는 공식
-> 왼쪽 바퀴 속도 = (2 * linear_vel - angular_vel * wheel_separation) / (2 * wheel_radius)
-> 오른쪽 바퀴 속도 = (2 * linear_vel + angular_vel * wheel_separation) / (2 * wheel_radius)

linear_vel: 로봇의 선속도
angular_vel: 로봇의 각속도
wheel_separation: 로봇의 두 바퀴 사이의 거리
wheel_radius: 로봇 바퀴의 반지름

- 현물 터틀봇의 analogWrite(MotorRFPWM,100)에서 OUTPUT 값 별 속도 구하고 해당하는 값을 라벨링함
- 결국 시물레이터에서 구하는 바퀴 속도랑 비교하여 그에 맞는 OUTPUT값을 주려고 함.

## 03/28 (금) : MQTT를 통해 ROS2와 통신 구현
- mqtt.ino 코드를 이용하여 ros2와 통신 할려고 했지만 실패 -> wifi 문제로 추정
- 개선 방안으로 노트북, 휴대폰 핫스팟 사용
- 노트북은 실패 하지만 휴대폰에서 호환성 최대화 한 후 코드 실행하면 hivemq와 통신 성공

### 다음주에 해야할 일
1. imu 센서로 속도 별 output 값 추정해야함
2. 1번을 활용하여 1차 함수식 만들기
3. v = r x w 공식을 활용하여 반지름 값을 구해서 코드안의 파라미터 바꿈
4. 통신으로 어떠한 형태로 데이터가 오는지 확인
5. 통신으로 온 데이터로 주행이 잘 되는지 확인
=======
# 맵 좌표
- 맵 크기
18.76 m × 26.20 m
- 출발지
(-50, -50.001)
- 게이트A
(-61.974, -53.235)
- 게이트 B
(-62/758, -43.768)
---------------------------------------
ros2 pkg executables advanced

## 빌드
```
rd /s /q build
rd /s /q install
rd /s /q log
colcon build --merge-install --symlink-install
# 빌드 후
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
```
### Error
- ssafy_bridge에서 나면 -> 실행중인 코드 전부 끄고 재빌드
- ssafy_msgs에서 나면 -> Cmake 패키지 에러

### pkg 확인
```
ros2 pkg executables 패키지이름
```

### 물건 놓기 강제 발행
```
# 맨 뒤에 -1 붙이면 한 번만 발행(될 때까지 발행해야 실행됨)
ros2 topic pub /hand_control ssafy_msgs/msg/HandControl "{control_mode: 1, put_distance: 0.0, put_height: 0.0}"
```

---

# 코드 실행 순서
## 1. 시뮬레이터 노드 실행 (/ssafy bridge)
```
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\src\ssafy_bridge\launch
ros2 launch ssafybridge_launch.py
```
## 2. SLAM 노드 실행 (/scan)
### 2.1. rqt 실행 
```
# 토픽 구독
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
rqt
```

### 2.2 rviz2 실행
```
# 목적지 찍는 용도
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
ros2 run rviz2 rviz2
```

### 3. 시나리오 코드 실행
```
path_tracking_astar_launch.py
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\src\sub2\launch
ros2 launch sub2 path_tracking_astar_launch.py
```

---


## etc1. SLAM 파일 실행
### 1.1. run_mapping.py
```
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws
call install\local_setup.bat
call C:\dev\ros2_eloquent\setup.bat
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\src\advanced\advanced
ros2 run advanced run_mapping
```

### 1.2. run_localization.py
```
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws
call install\local_setup.bat
call C:\dev\ros2_eloquent\setup.bat
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\src\advanced\advanced
ros2 run advanced run_localization
```

### 1.3 load_map.py
```
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws
call install\local_setup.bat
call C:\dev\ros2_eloquent\setup.bat
cd C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\src\advanced\advanced
ros2 run advanced load_map
```

## etc2. 개별 파일 실행
### 2.1. HW랑 MQTT 통신
```
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
ros2 run comm mqtt
```

### 2.2 백엔드랑 경로 주고 받기 (gate_receive.py)
```
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
ros2 run comm gate_receive
```

### 2.2 navigation_launch.py
```
call C:\dev\ros2_eloquent\setup.bat
call C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws\install\local_setup.bat
ros2 launch launch_package navigation_launch.py
```

------------------------------------------------------------------------
# 파일 구조
```
C:.
│  .gitignore
│
├─advanced # SLAM 패키지
│  │  package.xml
│  │  setup.cfg
│  │  setup.py
│  │
│  ├─advanced
│  │  │  load_map.py  # rviz2 상에 map 불러옴
│  │  │  map.png      
│  │  │  map_grid.txt
│  │  │  run_localization.py  # 출발지에서 목적지 rviz에서 표시했을 때 실제 로봇 주행을 rviz 상에서 확인할 수 있음
│  │  │  run_mapping.py       # 기본 SLAM 매핑 
│  │  │  __init__.py
│  │  │
│  │  └─__pycache__
│  │          load_map.cpython-37.pyc
│  │          mqtt.cpython-37.pyc
│  │          pjt.cpython-37.pyc
│  │          run_localization.cpython-37.pyc
│  │          run_mapping.cpython-37.pyc
│  │          __init__.cpython-37.pyc
│  │
│  ├─map  
│  │      map.txt # SLAM 완료 후 생성되는 txt 파일
│  │
│  ├─resource
│  │      advanced
│  │
│  └─test
│          test_copyright.py
│          test_flake8.py
│          test_pep257.py
│
├─comm  # MQTT 통신 패키지
│  │  CMakeLists.txt
│  │  package.xml
│  │
│  ├─include
│  │  └─comm
│  └─src
│          .env 
│          gate_receive.py    # 백엔드로부터 게이트 정보 받음
│          mqtt.py            # MQTT 기본 통신 코드
│          photo_send.py      # 사진 전송 코드
│          status_receive.py  # 현 터틀봇 상태 수신 (재가동, 멈춤 동작 실행 시)
│          status_send.py     # 현 터틀봇 상태 송신 
│
├─launch_package
│  │  package.xml
│  │  setup.cfg
│  │  setup.py
│  │
│  ├─launch
│  │      navigation_launch.py
│  │
│  ├─launch_package
│  │      __init__.py
│  │
│  ├─resource
│  │      launch_package
│  │
│  └─test
│          test_copyright.py
│          test_flake8.py
│          test_pep257.py
│
├─ssafy_bridge  # 시뮬레이터 통신 기본 패키지
│  │  package.xml
│  │  setup.cfg
│  │  setup.py
│  │
│  ├─launch
│  │  │  ssafybridge_launch.py
│  │  │
│  │  └─__pycache__
│  │          ssafybridge_launch.cpython-37.pyc
│  │
│  ├─resource
│  │      ssafy_bridge
│  │
│  ├─ssafy_bridge
│  │  │  cam_viewer.py
│  │  │  ssafy_udp_parser.py
│  │  │  sub_to_udp.py
│  │  │  udp_to_cam.py
│  │  │  udp_to_laser.py
│  │  │  udp_to_pub.py
│  │  │  utils.py
│  │  │  __init__.py
│  │  │
│  │  └─__pycache__
│  │          ssafy_udp_parser.cpython-37.pyc
│  │          sub_to_udp.cpython-37.pyc
│  │          udp_to_cam.cpython-37.pyc
│  │          udp_to_laser.cpython-37.pyc
│  │          udp_to_pub.cpython-37.pyc
│  │          utils.cpython-37.pyc
│  │          __init__.cpython-37.pyc
│  │
│  └─test
│          test_copyright.py
│          test_flake8.py
│          test_pep257.py
│
├─ssafy_msgs
│  │  CMakeLists.txt
│  │  package.xml
│  │
│  └─msg
│          BBox.msg
│          CustomObjectInfo.msg
│          EnviromentStatus.msg
│          HandControl.msg
│          Num.msg
│          ObjectInfo.msg
│          TurtlebotStatus.msg
│
├─sub1  # 시뮬레이터 상 센서, 카메라, 주행 관련 기본 스켈레톤 코드 기반 패키지 
│  │  package.xml
│  │  setup.cfg
│  │  setup.py
│  │
│  ├─launch
│  │      path_make_launch.py
│  │      path_tracking_launch.py
│  │      skeleton_launch.py
│  │
│  ├─path
│  │      map.txt
│  │      test.txt
│  │      test1.txt
│  │      test_origin.txt
│  │
│  ├─resource
│  │      my_package
│  │      sub1
│  │
│  ├─sub1
│  │  │  controller.py            # Automode로 주행 코드
│  │  │  controller_modified.py
│  │  │  handcontrol.py           # 물건 들고 내리기 
│  │  │  make_path.py             # 경로 생성
│  │  │  odom.py                  # 경로 주행
│  │  │  path_pub.py 
│  │  │  path_tracking.py         # 경로 따라가기
│  │  │  perception.py            # 카메라 local에서 창 열기
│  │  │  perception_ori.py
│  │  │  publisher.py             # 통신확인 코드 
│  │  │  subscriber.py            # 통신확인 코드 
│  │  │  __init__.py
│  │  │
│  │  ├─path
│  │  │      test.txt
│  │  │
│  │  └─__pycache__
│  │          controller.cpython-37.pyc
│  │          handcontrol.cpython-37.pyc
│  │          path_pub.cpython-37.pyc
│  │          perception.cpython-37.pyc
│  │          __init__.cpython-37.pyc
│  │
│  └─test
│          test_copyright.py
│          test_flake8.py
│          test_pep257.py
│   
└─sub2                                # 경로 관련 패키지 
    │  package.xml
    │  setup.cfg
    │  setup.py
    │
    ├─launch
    │      can_lift
    │      human_detector_launch.py
    │      path_tracking_astar_launch.py
    │      계속
    │      먼저
    │
    ├─map
    │      map.txt
    │
    ├─resource
    │      sub2
    │
    ├─sub2
    │  │  auto_goal_publisher.py        # 목적지 자동 설정(좌표 기반)
    │  │  a_star.py                    
    │  │  a_star_local_path.py
    │  │  a_star_skel.py
    │  │  ex_calib.py                   # LiDAR 센서, 카메라 병합 코드 
    │  │  human_detector.py             # 사람 감지 bbox 코드
    │  │  load_map.py
    │  │  odom.py
    │  │  path_tracking.py
    │  │  seg_binarizer.py
    │  │  utils.py
    │  │  __init__.py
    │  │
    │  └─__pycache__
    │          auto_goal_publisher.cpython-37.pyc
    │          a_star.cpython-37.pyc
    │          a_star_local_path.cpython-37.pyc
    │          load_map.cpython-37.pyc
    │          odom.cpython-37.pyc
    │          path_tracking.cpython-37.pyc
    │          __init__.cpython-37.pyc
    │
    └─test
            test_copyright.py
            test_flake8.py
            test_pep257.py
```
----------------------------------------------------------------------
### 250410
1. 하드웨어 통신 + 백엔드 통신 동시에 안되던 것 수정
- 각 노드마다 고유한 MQTT client id 생성 
- create_mqtt_client(suffix) 함수 사용 
```
# mqtt.py
import uuid  # 고유 ID 생성용
def create_mqtt_client(suffix=""):
    # 클라이언트 ID에 고유 suffix를 붙여 생성
    client_id = BASE_MQTT_CLIENT_ID + ("-" + suffix if suffix else "-" + str(uuid.uuid4()))
    client = mqtt.Client(client_id)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    return client
```
```
# gate_receive.py
# 고유한 MQTT 클라이언트 생성 (suffix "gate_receive")
mqtt_client = mqtt.create_mqtt_client("gate_receive")
```
```
# photo_send.py
# 고유한 MQTT 클라이언트 생성 (suffix "photo_send")
mqtt_client = mqtt.create_mqtt_client("photo_send")
```
2. 사진 전송 완료
![250410 사진 전송 완료](<img/250410 사진 전송 완료.png>)
- 기존 이미지만 보내지던 것에서 reservation_code까지 같이 보내지는 걸로 수정
- 기존 코드 문제점
  - gate_receive와 photo_send 간 퍼블리셔/서브스크라이버 QoS 추가 및 불일치
  - 10초 안에 메시지가 안들어올 경우 reservation_code 못 받은 채 종료됨 
  ```
- 수정 방향
  - 퍼블리셔-서브스크라이버 간 QoS 맞추기
  ```
  # gate_receive.py
  from rclpy.qos import QoSProfile, QoSDurabilityPolicy
  # TRANSIENT_LOCAL QoS 프로파일 생성: depth 1, durability TRANSIENT_LOCAL
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # "initial_goal" 토픽으로 메시지 publish (auto_goal_publisher가 구독)
        self.goal_pub = self.create_publisher(String, 'initial_goal', qos_profile)
  ```
  ```
  from rclpy.qos import QoSProfile, QoSDurabilityPolicy
  # photo_send.py
  qos_profile = QoSProfile(depth=1)
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(String, 'initial_goal', self.callback, qos_profile)
  ```
  - 대기 시간 20초로 늘림
  ```
  # photo_send.py
  def main(args=None):
  ...
  timeout = 20.0  # 최대 10초 대기
  ```

### 250409
1. 통신 테스트 백업
2. 통신 게이트 기반으로 auto_goal 수행 완료 백업 
3. 경로 와리가리 하는 거 수정 -> 속도 조금 늦춤
- SLAM 재 수행

### 250408 
1. comm 패키지의 통신 관련 코드 로그가 터미널 상에서 즉시 안나타남
-> MQTT 통신으로 게이트 정보를 받았음에도 불구하고 Ctrl C를 눌러야지만 받았는지 확인됨
```
# print() 호출에 flush 옵션 추가
# 출력 버퍼를 즉시 비움
print("photo send finish", flush=True)
```

### 250407 
1. 특정 구간에서만 카메라 센서가 켜짐
- 원인
  : 시뮬레이터 성능 문제 - 복잡한 가구 + 사람이 많은 B게이트 근처에서 카메라가 아예 안돌아감
- 해결방안
  - 게이트 근처에는 복잡한 에셋을 전부 지우고 새로 SLAM
  - 게이트와 멀리 있는 곳으로 에셋을 몰아서 공항 맵으로 보이게 수정함
2. perception.py 로직 수정
  -> control_mode 3번 발행 후 바로 저장 후 노드 종료
  - 카메라 프레임 저장 (바탕화면)
  - 저장 후 perception 노드 자동 종료
  - 종료 후 photo_send로 백엔드에 사진 데이터 전송하기 
  - 로컬에 저장된캡쳐된 이미지
  ![250407 시뮬레이터 상 로직 수행 완료](<img/250407 시뮬레이터 상 로직 수행 완료.jpg>)
3. hand_control 물건 내린 후 카메라 프레임 안으로 짐이 안들어옴
  - 물건 내리는 위치 수정
  ```
  self.hand_control_msg.put_distance = 2.0
  self.hand_control_msg.put_height = 0.0
  ```
  ![250407 물건내리면서 실시간 모니터링](<img/250407 물건내리면서 실시간 모니터링.png>)
4. 프론트엔드 통신 -> 안됨.
5. 데스크 안으로 LiDAR 센서가 통해서 길이 없음에도 불구 -> 길로 인식됨.
  - 데스크 안에 물건을 배치해서 SLAM되는 것 방지 및 들어가도 막힐 수 있도록 
  ![250407 SLAM 수정](<img/250407 SLAM 수정.png>)
  ![250407 벽뚫는거 수정](<img/250407 벽뚫는거 수정.png>)

### 250406
1. 물건 들고 내리기 완료(/handcontrol)
- 기존 코드의 문제점
  - 실제로 명령을 발행하는 것(control_mode)과 명령이 반영되는 것에 지연이 발생함.
  
- 수정한 부분
  - has_object 변수를 사용해 물건 보유 여부를 추적 + pick‑up(2번)과 drop‑off(3번)이 10회 연속 발행되면 강제 전환되도록 했음
  - -> 기존 코드의 has_object로 물건을 들고 있는 것을 따로 Bool로 받았는데, 이 부분을 완전히 지우고 센서 상태(can_lift, can_put)만으로 물건 여부를 판단
  - 도착지 부분을 path_tracking 코드에서 로직을 가져와서 반영함
  - 이전 control_mode와의 비교 없이 새 명령을 매 타이머 주기마다 발행 -> 명령 발행 로직 간소화함 
- 결과
![250406 출발지 및 경로 수행 중_ 물건들기](<img/250406 출발지 및 경로 수행 중_ 물건들기.png>)
![250406 목적지_물건내리기](<img/250406 목적지_물건내리기.png>)

### 250404
1. 목적지 근사치 도달 시 로그 멈춤
- 기존 목적지 도달 -> 근사치 도달. 소수점이 계속 변하면서 움직임은 없지만 로그가 계속 뜸 
![250404 기존 목적지 도달](<img/250404 기존 목적지 도달.png>)
- 수정한 부분 
![250404 목적지도달](<img/250404 목적지도달.png>)

2. goal_pose 안찍고도 자동 도달
![250404 게이트 자동 도달](<img/250404 게이트 자동 도달.png>)

3. 통신 패키지 새로 빌드 (comm)
- 파일 구조
```
C:.
│  CMakeLists.txt
│  package.xml
│
├─include
│  └─comm
└─src
        .env
        gate_receive.py
        mqtt.py
        photo_send.py
        status_receive.py
        status_send.py
```
```
# 파일에 추가한 내용 -> env 경로 수정
from pathlib import Path

# mqtt.py가 있는 디렉토리의 .env 파일 경로 지정 및 로딩
env_path = Path(__file__).resolve().parent / '.env'
load_dotenv(dotenv_path=env_path)
```
![250404 MQTT 백엔드 통신 커넥트](<img/250404 MQTT 백엔드 통신 커넥트.png>)

### 250403
1. HW MQTT 통신 + a_star 같이 돌리니깐 경로가 꼬임
- 기존 cmd_vel를 구독했을 때 
-> cmd_vel이 바뀌는 값을 정확히 읽어내지못함 (-1~1 사이로만 바뀜)
![250403 cmd_vel 토픽](<img/250403 cmd_vel 토픽.png>)
```
# /cmd_vel 토픽 구독 (메시지 타입: Twist)
self.subscription = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.cmd_vel_callback,
    10)
self.subscription  # 변수 사용 경고 방지
```
- imu 토픽 구독으로 변경
실제 발행되는 imu 토픽
![img/250403 cmd_vel 토픽.png](<img/250403 cmd_vel 토픽.png>)
MQTT에서 imu토픽 구독하니깐 바뀐 상태
![img/250403 imu_cmd_vel발행해서바뀜.png](<img/250403 imu_cmd_vel발행해서바뀜.png>)

-> MQTT 먼저 실행 시: 경로 주행 X
-> path tracking 먼저 실행 후 MQTT 실행 시: 경로 꼬임
-> 코드 추가 후 launch 파일에서 코드 동시에 수행됨 + 하드웨어 주행까지 확인 완료

- MQTT 네트워크 루프를 백그라운드에서 실행
```
self.client.loop_start()
```

### 250402
1. 커스텀 SLAM 완성
```
"MAP_CENTER": (-50,-50.001),
"MAP_SIZE": (30, 30),
```
```
m.origin.position.x = params_map["MAP_CENTER"][0] - params_map["MAP_SIZE"][0]/2
m.origin.position.y = params_map["MAP_CENTER"][1] - params_map["MAP_SIZE"][1]/2
```
![img/250402 SLAM_finish.png](<img/250402 SLAM_finish.png>)
- load_map
```
map_size_x= 600
map_size_y= 600

self.map_offset_x=-50-15.0
self.map_offset_y=-50-15.0
```
![img/250402 rviz_SLAM.png](<img/250402 rviz_SLAM.png>)
- 맵 좌표 기반 SLAM
![img/250402 rviz.png](<img/250402 rviz.png>)
- 최종 맵 (기존 맵은 의자 에셋 -> 실제로 가지못하는 길인데 SLAM 상으로는 경로가 나오는 문제)
![img/250402 맵 최종.png](<img/250402 맵 최종.png>)

2. a-star 따라서 로봇 동작 수행
```
# 로직 2
self.goal = [184, 224]
self.map_size_x = 600
self.map_size_y = 600
self.map_resolution = 0.05
self.map_offset_x = -65.0
self.map_offset_y = -65.001
```
![img/250402 path.png](<img/250402 path.png>)
3. 물건 들고 경로 수행
- ~\sub2\launch\path_tracking_astar_launch.py 에 handcontrol 로직 추가
- 출발 시 물건있고 + can_put일 경우 물건 들도록 handcontrol 코드 수정 
![img/250402 hand_control_path.png](<img/250402 hand_control_path.png>)

### 240401
1. 맵 크기에 비해 데이터가 너무 방대함 + 좌표값 정확하지 않음
- 120 x 120 사이즈
![img/250401 기존_SLAM.png](<img/250401 기존_SLAM.png>)
- run_mapping.py 수정 "MAP_CENTER": (-50.00307846,-50.06665421)
![img/250401 SLAM_수정중.png](<img/250401 SLAM_수정중.png>)

### 350331
1. SLAM map.txt 추출 완료 
- ~\smarthome-resources\ros2_ws\src\advanced\map\map.txt
 - load_map
![img/250331 load_SLAM.png](<img/250331 load_SLAM.png>)
 - SLAM 과정
![img/250331 custom_SLAM_testing.png](<img/250331 custom_SLAM_testing.png>)

2. 커스텀 맵에서 perception 노드 수행 및 사진 저장 완료
- perception 노드 수행
```
class IMGParser(Node):

    def __init__(self):
        # node_name 기본 맵이랑 수정 - 충돌 방지 
        super().__init__(node_name='image_convertor_custom')
```
- 사진 저장 
![img/250331 perception_monitoring.png](<img/250331 perception_monitoring.png>)
- 바탕화면에 저장되도록 함. -> 추후 수정 가능
![img/250331 captured_image.jpg](<img/250331 captured_image.jpg>)

### 250328
1. perception 로직 수정해야 함
- cam_viewer.py, udp_to_cam.py 수정 필요
```
C:.
│  package.xml
│  setup.cfg
│  setup.py
│
├─launch
│  │  ssafybridge_launch.py
│  │
│  └─__pycache__
│          ssafybridge_launch.cpython-37.pyc
│
├─resource
│      ssafy_bridge
│
├─ssafy_bridge
│  │  cam_viewer.py
│  │  ssafy_udp_parser.py
│  │  sub_to_udp.py
│  │  udp_to_cam.py
│  │  udp_to_laser.py
│  │  udp_to_pub.py
│  │  utils.py
│  │  __init__.py
│  │
│  └─__pycache__
│          ssafy_udp_parser.cpython-37.pyc
│          sub_to_udp.cpython-37.pyc
│          udp_to_cam.cpython-37.pyc
│          udp_to_laser.cpython-37.pyc
│          udp_to_pub.cpython-37.pyc
│          utils.cpython-37.pyc
│          __init__.cpython-37.pyc
│
└─test
        test_copyright.py
        test_flake8.py
        test_pep257.py
```
2. 경로 테스트 
![img/250328 a_star_test.png](<img/250328 a_star_test.png>)

### 250327
1. 물건 들고 옮기기 로직 수정
2. 커스텀 맵 SLAM
- 맵 크기 조정 (50,50) ~ (100, 100) 사이에서 조정 필요
3. 사진 찍기 /perception 윈도우에서 창이 안열림 -> 코드 수정중

### 250326
1. 커스텀 맵 SLAM 구현
- SLAM toolboox + NAV2 기반 자율주행 SLAM 구현해보려했으나 빌드 오류(cmake 오류) -> 명세서 기반 수동 SLAM 구현
```
# 오류내역
# 해결 안돼서 빌드 로그 삭제 후 기존대로 수동 SLAM 진행 

C:\Users\SSAFY\Desktop\smarthome-resources\ros2_ws>colcon build --merge-install --symlink-install --cmake-args -DCMAKE_TOOLCHAIN_FILE=C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake -DTINYXML2_INCLUDE_DIR=C:/dev/vcpkg/installed/x64-windows/include -DTINYXML2_LIBRARY=C:/dev/vcpkg/installed/x64-windows/lib/tinyxml2.lib
Starting >>> nav2_common
Starting >>> nav_2d_msgs
Starting >>> ssafy_msgs
Starting >>> nav2_gazebo_spawner
Starting >>> ssafy_bridge
Starting >>> sub1
Starting >>> sub2
Starting >>> sub3
Finished <<< sub2 [4.08s]
Finished <<< sub3 [4.08s]
Finished <<< nav2_gazebo_spawner [4.50s]
Finished <<< ssafy_bridge [4.47s]
Finished <<< sub1 [4.45s]
--- stderr: nav2_common
CMake Warning:
  Manually-specified variables were not used by the project:

    TINYXML2_INCLUDE_DIR
    TINYXML2_LIBRARY


---
Finished <<< nav2_common [5.66s]
Starting >>> nav2_msgs
Starting >>> nav2_voxel_grid
Finished <<< ssafy_msgs [59.3s]
Starting >>> advanced
Finished <<< advanced [0.05s]
--- stderr: nav2_voxel_grid
CMake Deprecation Warning at C:/dev/ros2_eloquent/src/gtest_vendor/CMakeLists.txt:2 (cmake_minimum_required):
  Compatibility with CMake < 2.8.12 will be removed from a future version of
  CMake.

  Update the VERSION argument <min> value or use a ...<max> suffix to tell
  CMake that the project does not need compatibility with older versions.


CMake Warning:
  Manually-specified variables were not used by the project:

    TINYXML2_INCLUDE_DIR
    TINYXML2_LIBRARY


---
Failed   <<< nav2_voxel_grid [53.9s, exited with code 1]
Aborted  <<< nav_2d_msgs [1min 31s]
Aborted  <<< nav2_msgs [2min 19s]

Summary: 8 packages finished [2min 25s]
  1 package failed: nav2_voxel_grid
  2 packages aborted: nav2_msgs nav_2d_msgs
  4 packages had stderr output: nav2_common nav2_msgs nav2_voxel_grid nav_2d_msgs
  25 packages not processed
WNDPROC return value cannot be converted to LRESULT
TypeError: WPARAM is simple, so must be an int object (got NoneType)
```
2. SLAM 이후 map.txt 추출
- 100에 가까울 수록 장애물이 존재할 가능성이 있다는 뜻임 (0~100)
- 맵 : 350*350 = 122,500개의 값
    - 해상도: grid 한 칸당 0.05m
    - 오프셋은 맵의 우측하단 끝이 기준좌표계(map 좌표계)로부터 떨어져 있는 거리
        - x축 오프셋: -16.75m
        - y축 오프셋: -12.75m
    - 읽어와서 사용할 때는 numpy를 이용해 2차원 배열 형태로 사용함 

### 250325
1. 명세서 기반 SLAM 구현
![img/250325 SLAM_test.png](<img/250325 SLAM_test.png>)
- Windows에서 진행(Ubuntu에서는 센서 데이터가 처리되지 않는 오류 발생 Why? 시뮬 실행은 윈도우에서 하고 ROS는 우분투에서 해서 인건지 잘 모르겠음)
- colcon build 오류 해결 -> 파일명을 기존 catkin_ws -> ros2_ws로 변경
```
colcon build --merge-install --symlink-install
```
>>>>>>> master
