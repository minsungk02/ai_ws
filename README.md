# ai_ws – Unified Autonomy Workspace

이 워크스페이스는 Jetson 기반 RC카에서 카메라·라이다 인지와 아두이노 제어를 일원화하기 위한 ROS1(catkin) 환경입니다. 기존에 보유한 코드베이스(CNN 차선 인식, OpenCV 차선 추종, YOLO 객체 인식, 라이다 회피, 아두이노 펌웨어)를 패키지 단위로 정리해 한 런치로 구동하는 것이 목표입니다.

## 디렉터리 개요

```
ai_ws/
├── build, devel, install      # catkin_make 결과물
├── src/
│   ├── autocar_bringup        # 전체 런치/파라미터 집합
│   ├── common_serial          # /dev/arduino 프로토콜 · 유틸
│   ├── fusion_control         # 센서 융합 + 속도/조향 의사결정 노드
│   ├── perception_lane_cnn    # CNN_model 실시간 추론 노드
│   ├── perception_lidar       # rplidar_ros 데이터를 융합 토픽으로 변환
│   └── perception_object      # Ultralytics YOLO 기반 객체/신호등 인식 노드
└── README.md                  # (본 문서)
```

- `common_serial`: 현재 `prac_src_str/01-1_lane_following_simple.py`와 `prac_src/rplidar_ros/*.py` 등에 중복된 `create_command()`·시리얼 초기화를 파이썬 모듈로 추출하여 모든 노드가 재사용하도록 합니다. `/dev/arduino` 심볼릭 링크 설정과 보드 속도를 여기에서 통일합니다.
- `perception_lane_cnn`: 패키지 내부(`perception_lane_cnn/cnn_model`)의 `model.py`와 `weights/best_model.pth`를 로드해 카메라 토픽을 구독하고 `lane/steering_cnn`을 발행합니다. 필요 시 `~serial_output` 파라미터를 켜면 단독 주행도 가능합니다.
- `perception_object`: Ultralytics YOLO( v8/YOLO11 계열) 파이썬 API를 사용해 `/detections/yolo`를 발행합니다. 기본 가중치는 `~/ai_ws/models/best.pt`이며, `~weights` 파라미터로 다른 `.pt`를 지정해 곧바로 교체할 수 있습니다. (순정 YOLOv5 레포의 `detect.py`에 넣으면 `state_dict` mismatch가 날 수 있으니 Ultralytics 방식으로만 사용하세요.)
- `perception_lidar`: 기존 `lidar_ros_simple.py` 로직을 ROS 노드화하여 `/scan`을 구독하고, 장애물 요약/권장 속도/권장 조향을 `LidarState` 메시지로 발행합니다. 필요 시 단독 주행 모드에서도 시리얼 명령을 낼 수 있습니다.
- `fusion_control`: 차선, 객체, 라이다 토픽을 모두 구독해 상태 머신/안전 로직을 실행하고, `common_serial`을 이용해 아두이노로 조향/속도를 송신합니다. `/fusion/command`에는 `Twist` 형식으로도 명령을 퍼블리시하여 로깅이 쉽습니다.
- `autocar_bringup`: `launch/autocar.launch`에 모든 노드를 묶어두었습니다. `roslaunch autocar_bringup autocar.launch`만 실행하면 카메라, 라이다, 차선, 객체, 융합 노드가 동시에 올라옵니다(라이다 드라이버는 기존 `catkin_ws`에서 별도 실행).

## 외부 자원 연결

| 역할 | 기존 위치 | ai_ws 연동 방법 |
| --- | --- | --- |
| CNN 학습/가중치 | `/home/huins/CNN_model` | `perception_lane_cnn` 노드에서 `sys.path` 추가 또는 패키지 설치 형태로 import |
| 라이다 회피 예제 | `/home/huins/prac_src/rplidar_ros` 및 `~/catkin_ws` | rplidar 드라이버는 기존 워크스페이스에서 실행하고, `perception_lidar`가 `/scan`을 구독 |
| YOLO/신호등 | 학습/실험용 코드: `/home/huins/yolov5`, <br>런타임 가중치: `~/ai_ws/models/best.pt`(Ultralytics) | 추론은 `perception_object`에서 Ultralytics API로 수행. 학습 레포에서 만든 `.pt`를 이 경로로 복사하면 즉시 사용 가능 |
| 아두이노 펌웨어 | `/home/huins/prac_src1/AI_CAR_Steering` | 시리얼 프로토콜은 `common_serial`에서 재사용, 펌웨어는 동일 |

필요하다면 위 디렉토리를 `ai_ws/external/` 하위에 심볼릭 링크하여 버전 관리할 수 있습니다.

## 빌드 및 환경 설정

```bash
# 1) ROS 환경 로드
source /opt/ros/noetic/setup.bash

# 2) 워크스페이스 빌드
cd ~/ai_ws
catkin_make

# 3) 개발 환경에 추가
echo "source ~/ai_ws/devel/setup.bash" >> ~/.bashrc
```

현재 `common_serial`, `perception_lane_cnn`, `perception_lidar`, `perception_object`, `fusion_control`, `autocar_bringup`에 기본 노드/메시지가 모두 작성되어 있어 `catkin_make` 후 즉시 실행 및 커스터마이징이 가능합니다. 특히 `perception_object`는 학습된 YOLO 가중치 파일만 교체하면 곧바로 새 환경을 반영할 수 있게 설계했습니다.

## 다음 단계 체크리스트

1. (완료) `common_serial/scripts/serial_bridge.py`로 모든 시리얼 명령을 공용화했습니다.
2. (완료) `perception_lane_cnn/scripts/lane_cnn_node.py`가 카메라 토픽을 받아 CNN 추론 → `lane/steering_cnn` 퍼블리시까지 처리합니다.
3. (완료) `perception_object/scripts/object_yolo_node.py`는 Ultralytics YOLO를 로드해 `DetectionArray` 토픽을 발행합니다. 파인튜닝한 `.pt` 파일을 `~weights`로 지정해 곧바로 교체할 수 있습니다.
4. (완료) `perception_lidar/scripts/lidar_state_node.py`가 `/scan`을 요약해 `LidarState` 토픽을 제공합니다.
5. (완료) `fusion_control/scripts/fusion_node.py`가 차선/라이다/객체 정보를 융합해 최종 조향·속도 명령을 계산하고 아두이노로 전송합니다.
6. (완료) `autocar_bringup/launch/autocar.launch`로 모든 노드를 한 번에 실행할 수 있습니다. 필요한 경우 라이다 드라이버(`rplidar_ros`)를 포함하도록 추가하면 됩니다.

이 구조를 기반으로 차선(CNN), 객체(YOLO), 라이다 센서 퓨전을 하나의 워크스페이스에서 관리하고, 필요 시 기존 `~/catkin_ws`는 라이다 드라이버 전용 백업으로 유지할 수 있습니다.

## 실행 절차 (카메라 + RPLIDAR + CNN + YOLO + 아두이노)

1) 빌드/환경 로드
```bash
source /opt/ros/noetic/setup.bash
cd ~/ai_ws && catkin_make
source ~/ai_ws/devel/setup.bash
```

2) 터미널 1 – 라이다 드라이버(기존 워크스페이스)
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rplidar_ros rplidar_a1.launch
# 확인: rostopic echo /scan  또는  rostopic hz /scan
```

3) 터미널 2 – ai_ws 전체 런치
```bash
source /opt/ros/noetic/setup.bash
source ~/ai_ws/devel/setup.bash
export YOLOv5_AUTOINSTALL=0
roslaunch autocar_bringup autocar.launch serial_port:=/dev/arduino
```
기본으로 간단 카메라 퍼블리셔(`simple_cam_pub.py`)가 `/camera/image_raw`를 올립니다. 이미 다른 카메라 드라이버가 있을 경우 `enable_cam_pub:=false`로 끄고 사용하세요.
YOLO 가중치는 `~/ai_ws/models/best.pt`(Ultralytics)로 기본 설정되어 있고, `conf_threshold`/`iou_threshold`/`target_labels`를 런치 인자에서 바로 조정할 수 있습니다. 예) 낮은 점수까지 보기: `roslaunch autocar_bringup autocar.launch conf_threshold:=0.01 target_labels:="[]"`.

## 실행 후 토픽/상태 확인

- 기본 토픽 리스트 필터:
  ```bash
  rostopic list | egrep 'lane/steering_cnn|detections/yolo|fusion/command|lidar/state|camera'
  ```
- 퍼블리시 확인:
  ```bash
  rostopic hz /camera/image_raw       # 카메라 fps
  rostopic hz /lane/steering_cnn      # CNN 조향 주기
  rostopic echo /lane/steering_cnn    # 조향 값
  rostopic echo /detections/yolo      # YOLO 감지
  rostopic echo /fusion/command       # 최종 Twist
  ```
- 시각화:
  - `rqt_image_view` → `/camera/image_raw` 선택 (SSH 무X11 환경이면 실행 불가)
  - `rqt_graph`로 노드/토픽 연결 확인
기본값: 카메라 `/camera/image_raw`, CNN 가중치 `/home/huins/CNN_model/train/exp1/best_model.pth`, YOLO 가중치 `/home/huins/ai_ws/models/best.pt`(Ultralytics), `conf=0.10`, `iou=0.45`, `target_labels=['person','stop_sign','red','yellow','green']`, 시리얼 115200. 콘솔에 `SerialBridge connected to /dev/arduino @ 115200`가 보여야 합니다.

4) 선택 확인/모니터링
- `rostopic echo /fusion/command`로 최종 조향/속도(Twist) 확인
- `rostopic echo /lane/steering_cnn`로 CNN 조향만 확인
- `miniterm /dev/ttyUSB0 115200`은 roslaunch와 포트 충돌하므로 종료 후 단독 확인

## 객체 인식 트러블슈팅 (raw=0일 때)

- 프레임 저장: `rosrun image_view image_saver image:=/camera/image_raw _filename_format:=/tmp/frame.jpg` 후 `/tmp/frame.jpg`를 직접 확인합니다.
- 저장된 프레임에 모델 직접 적용(ROS 불필요):
  ```bash
  LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 \
  yolo predict model=/home/huins/ai_ws/models/best.pt source=/tmp/frame.jpg imgsz=640 conf=0.01 device=0 save=True
  ```
  `runs/detect/predict/` 안 결과 이미지에 박스가 그려지면 모델은 정상이며, ROS 경로/파라미터 문제일 가능성이 큽니다.
- ROS 노드에서만 `raw=0`이면 `conf_threshold`를 더 낮추고 필터링을 꺼서 확인:
  ```bash
  roslaunch autocar_bringup autocar.launch serial_port:=/dev/arduino conf_threshold:=0.01 target_labels:="[]"
  ```
- CLI에서도 박스가 0이면 그 프레임 조건(노출/거리/각도/학습 데이터와의 차이)이 원인입니다. 카메라 설정을 바꾸거나 해당 환경으로 재학습이 필요합니다.

## 런타임 모니터링

ROS가 올라온 뒤 토픽과 영상/스캔을 빠르게 확인하는 방법:

- 텍스트로 토픽 확인:
  ```bash
  rostopic echo /fusion/command      # 최종 Twist 명령
  rostopic echo /lane/steering_cnn   # CNN 조향 추정(Float32)
  rostopic echo /lidar/state         # 라이다 요약
  rostopic echo /detections/yolo     # YOLO 감지 결과
  rostopic echo /scan                # 라이다 원시 스캔
  ```
- 실시간 영상: `rqt_image_view` 실행 후 `/camera/image_raw` 선택.
- RViz 예시 설정:
  - Add → LaserScan: `/scan`
  - Add → Image: `/camera/image_raw`
  - 명령 값은 `rostopic echo /fusion/command`로 수치 확인(별도 Marker 변환 노드는 없음).

## 현재 안정 실행 파라미터 메모

- 환경변수(스레드 제한): `export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 YOLOv5_AUTOINSTALL=0`
- 카메라 부하 축소: `camera_fps:=15 camera_width:=640 camera_height:=480`
- YOLO: `yolo_use_cuda:=true yolo_use_fp16:=true img_size:=416 conf_threshold:=0.10`
- Lane CNN: `lane_use_cuda:=false` (필요하면 frame_skip=1 추가로 CPU 부하 감소)
- 실행 예:
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/ai_ws/devel/setup.bash
  export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 YOLOv5_AUTOINSTALL=0
  roslaunch autocar_bringup autocar.launch \
    serial_port:=/dev/arduino \
    camera_fps:=25 camera_width:=640 camera_height:=480 \
    yolo_use_cuda:=true yolo_use_fp16:=true img_size:=512 conf_threshold:=0.50 \
    lane_use_cuda:=true
  ```
