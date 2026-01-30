# 🤖 Isaac Sim & YOLOv8-OBB based 3D Pose Correction System

![Project Banner](https://placeholder.com/wp-content/uploads/2018/10/placeholder.com-logo1.png)

<br>

## 💻 Tech Stack

| Category | Technology |
|:---:|:---|
| **Simulation** | ![IsaacSim](https://img.shields.io/badge/NVIDIA-Isaac%20Sim-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![Omniverse](https://img.shields.io/badge/NVIDIA-Omniverse-76B900?style=for-the-badge&logo=nvidia&logoColor=white) |
| **OS** | ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-E95420?style=for-the-badge&logo=ubuntu&logoColor=white) |
| **Middleware** | ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white) |
| **AI / Vision** | ![YOLOv8](https://img.shields.io/badge/YOLO-v8%20OBB-00FFFF?style=for-the-badge) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white) |
| **Hardware** | ![Doosan](https://img.shields.io/badge/Doosan-M0609-000000?style=for-the-badge) ![RealSense](https://img.shields.io/badge/Intel-RealSense%20D455-0071C5?style=for-the-badge&logo=intel&logoColor=white) |
| **Language** | ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white) |

<br>

---

## 🔍 Project Overview
스마트 팩토리 공정에서 컨베이어 벨트 위의 부품이 미세하게 틀어지거나(Orientation Error) 뒤집혀 발생하는 병목 현상을 해결하기 위한 **Digital Twin 기반 로봇 제어 시스템**입니다. 

실제 환경을 **NVIDIA Isaac Sim**으로 완벽하게 구현하고, **YOLOv8-OBB**를 통해 객체의 회전 각도까지 정밀하게 인식하여 로봇이 자동으로 정렬(Pick & Place)하는 자동화 프로세스를 구축했습니다.

<br>

## 👥 Team & Roles

| Name | Role | Responsibility |
|:---:|:---:|:---|
| **Kim Jung-wook** | Team Leader <br> Robotics Engineer | - **Isaac Sim Environment Setup:** 실제 공장 환경(조명, 컨베이어, 로봇) Digital Twin 구축 <br> - **Robot Manipulation:** ROS2 기반 제어 노드 작성 및 Pick & Place 모션 플래닝 <br> - **System Integration:** Vision 데이터와 Robot Control 간 통신 최적화 |
| **Lee Hyo-won** | AI & System Engineer | - **YOLO Training:** Custom Dataset을 활용한 YOLOv8-OBB 모델 학습 및 튜닝 <br> - **ROS Integration:** AI 추론 결과(B-Box, Angle)를 ROS2 토픽으로 발행(Publish) |
| **Kim Da-bin** | Data Engineer & PM | - **Data Pipeline:** Roboflow 활용 학습 데이터셋 구축 및 레이블링(Labeling) <br> - **Documentation:** 산출물 관리, 발표 자료 및 시연 시나리오 기획 |

<br>

## 🛠 System Architecture

[Image of System Architecture Diagram]

---

### 📡 Communication Flow Chart
> **[여기에 Flow Chart 스크린샷 이미지 삽입]**  
> 예: `assets/flowchart.png`

---

### 🧩 ROS2 Architecture Structure
> **[여기에 ROS2 아키텍처 구조도 스크린샷 이미지 삽입]**  
> 예: `assets/architecture.png`

---

이 시스템은 크게 **인지(Perception)**, **판단(Decision)**, **제어(Control)** 3단계로 구성됩니다.

1. **Input:** RealSense Depth Camera를 통해 RGB 및 Depth 데이터 수집  
2. **3D Pose Estimation:** YOLOv8-OBB로 객체의 2D 좌표와 기울기(θ)를 검출하고 Depth 정보를 결합하여 3D 좌표(X, Y, Z)로 변환  
3. **Robot Control:** 보정이 필요한 각도가 감지되면 로봇이 해당 좌표로 이동하여 부품을 정렬  

---

## 💻 Tech Stack

| Category | Technology |
| :---: | :--- |
| **Simulation** | NVIDIA Isaac Sim / Omniverse |
| **OS / Middleware** | Ubuntu 22.04 / ROS2 Humble |
| **AI / Vision** | YOLOv8-OBB / OpenCV / PyTorch |
| **Hardware** | Doosan M0609 / Intel RealSense D455 |
| **Language** | Python 3.10 |

<br>

## 🚀 Key Features & Logic

### 0. ROS2 Node Composition (Perception → Decision → Control)

#### 🔹 obb_node.py (Perception + Decision)
- YOLOv8-OBB 기반 Oriented Bounding Box 검출
- Depth + CameraInfo를 활용한 3D Pose 계산
- Yaw 각도 기반 OK / DEFECT 판정
- **Debounce 로직**을 통해 오탐 감소
- 불량 확정 시 `/target_pose` **Latch Publish**
- RViz 시각화를 위한 `/target_marker`, `/object_marker` 발행

#### 🔹 move_joint.py (Control)
- `/moverobot`, `/target_pose` 구독
- **Approach → Pick → Retreat** 로봇 모션 시퀀스 실행
- `/joint_command` (JointState) 발행
- 발표/연출 목적의 `/gripper_close` Bool 토픽 발행  
  *(실제 하드웨어 환경에서는 그리퍼 드라이버로 연결 가능)*

---

### ▶ Run Instructions (Parameter Tuning)

```bash
# obb node
ros2 run my_examples obb_node --ros-args \
  -p model_path:=/home/rokey/ros2_ws/best.pt \
  -p defect_need:=5 -p ok_need:=5 \
  -p minangle_deg:=10.0 -p maxangle_deg:=70.0

# move node
ros2 run my_examples move_joint --ros-args \
  -p approach_sec:=1.2 -p pick_sec:=1.2 -p retreat_sec:=1.2 \
  -p hold_after_retreat:=true \
  -p hint_gain:=0.6

# pose tuning (radian)
ros2 run my_examples move_joint --ros-args \
  -p pick_pose:="[1.6, -0.8, 1.35, -1.25, 1.60, 0.0]"

---

✅ [추가] 주요 파라미터 요약

defect_need, ok_need: 불량/정상 상태 전환을 위한 연속 프레임 수(디바운스)

minangle_deg, maxangle_deg: OK로 판단할 yaw 각도 범위

approach_sec, pick_sec, retreat_sec: 시퀀스 단계별 유지 시간

pick_pose, approach_pose, retreat_pose: 모션 시퀀스의 joint 목표(라디안)

hint_gain: /target_pose 기반으로 joint_1을 살짝 보정하는 “시각적 반응성” 연출 파라미터

<br>
1. Oriented Bounding Box (OBB) Detection

기존의 수평적인 Bounding Box(AABB)는 회전된 물체의 정확한 각도를 알 수 없는 한계가 있었습니다. 이를 극복하기 위해 YOLOv8-OBB 모델을 도입하여 물체의 Heading Angle(Yaw) 값을 실시간으로 추론했습니다.

[여기에 PPT 17페이지의 YOLO 탐지 결과(초록색 박스 쳐진 것) 이미지를 넣으세요]

✅ [추가] Key Point

OBB의 theta(radian) 값을 활용해 물체의 yaw를 추정하고, OK/DEFECT 판정 기준으로 사용합니다.

단일 프레임 오탐을 줄이기 위해 연속 프레임 디바운스를 적용합니다.

2. 3D Coordinate Conversion (Deprojection)

2D 이미지 상의 픽셀 좌표 $(u, v)$를 3D 로봇 좌표계 $(x, y, z)$로 변환하기 위해 핀홀 카메라 모델을 적용했습니다.

𝑋
=
(
𝑢
−
𝑐
𝑥
)
×
𝑍
/
𝑓
𝑥
𝑌
=
(
𝑣
−
𝑐
𝑦
)
×
𝑍
/
𝑓
𝑦
X=(u−c
x
	​

)×Z/f
x
	​

Y=(v−c
y
	​

)×Z/f
y
	​


$Z$: Depth Map에서 추출한 심도 값

$f_x, f_y$: 카메라 초점 거리 (Focal Length)

$c_x, c_y$: 주점 (Principal Point)

✅ [추가] Key Point

검출된 픽셀 중심점과 Depth를 결합해 3D 좌표를 계산하고 /object_pose로 발행합니다.

불량 확정 순간의 pose는 /target_pose로 1회 고정(latch) 발행하여 제어의 입력을 안정화합니다.

3. Digital Twin Simulation

물리 엔진이 적용된 Isaac Sim 환경에서 컨베이어 벨트의 마찰력과 로봇의 동역학을 시뮬레이션하여, 실제 현장 도입 시 발생할 수 있는 시행착오를 최소화했습니다.

[여기에 PPT 10페이지나 11페이지의 시뮬레이션 환경 캡처를 넣으세요]

✅ [추가] (선택) RViz Visualization

/object_marker (CUBE): 실시간 감지 물체의 위치/자세 시각화(초록 박스)

/target_marker (SPHERE): 불량 확정 순간의 목표 지점 시각화(빨간 점)

/target_pose: 목표 pose 좌표축 표시

<br>
📊 Project Results

[cite_start]Detection Accuracy: mAP50-95 기준 90% 이상 달성 [cite: 140]

[cite_start]Pose Estimation Error: 평균 오차 5도 내외로 정밀 보정 성공 [cite: 382]

Impact: 불량 부품의 자동 재정렬을 통해 공정 병목 현상 해소 및 생산 효율 증대 기대

<br>
🎥 Demo Video

[여기에 시연 영상 GIF나 유튜브 링크를 넣으면 완벽합니다!]
