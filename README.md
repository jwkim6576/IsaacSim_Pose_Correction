# 🤖 Isaac Sim & YOLOv8-OBB based 3D Pose Correction System

![Project Banner](https://placeholder.com/wp-content/uploads/2018/10/placeholder.com-logo1.png)

<br>

## 🗂️ 목차

### 1. [Project Overview](#-project-overview)
### 2. [Team & Roles](#-team--roles)
### 3. [System Architecture](#-system-architecture)
### 4. [Tech Stack](#-tech-stack)
### 5. [Key Features & Logic](#-key-features--logic)
### 6. [Project Results](#-project-results)
### 7. [Demo Video](#-demo-video)

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
| **Hardware** | UR10 / Doosan M0609 / Intel RealSense D455 |
| **Language** | Python 3.10 / C++ |

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

## 🔧 주요 파라미터 설명

| Parameter | Description |
|----------|-------------|
| `defect_need / ok_need` | 불량/정상 상태 전환을 위한 연속 프레임 수 |
| `minangle_deg / maxangle_deg` | OK 판정 Yaw 각도 범위 |
| `approach_sec / pick_sec / retreat_sec` | 모션 시퀀스 단계별 유지 시간 |
| `pick_pose / approach_pose` | 로봇 관절 목표 각도 (라디안) |
| `hint_gain` | `/target_pose` 기반 joint_1 보정 강도 |

---

## 1. Oriented Bounding Box (OBB) Detection

YOLOv8-OBB를 사용하여 물체의 **Heading Angle (Yaw)** 까지 추정 가능합니다.

> **[여기에 YOLO OBB Detection 결과 스크린샷 삽입]**

---

## 2. 3D Coordinate Conversion (Deprojection)

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy

- **Z**: Depth Map에서 추출한 거리 값  
- **fx, fy**: 카메라 초점 거리 (Focal Length)  
- **cx, cy**: 주점 좌표 (Principal Point)

---

## 3. Digital Twin Simulation

Isaac Sim 환경에서 실제 공정을 시뮬레이션하여 현실 적용 시 발생할 수 있는 시행착오를 최소화합니다.

> **[여기에 IsaacSim 환경 스크린샷 삽입]**

---

## 📊 Project Results

- **Detection Accuracy:** mAP50-95 기준 **90% 이상**  
- **Pose Estimation Error:** 평균 오차 **5도 내외**  
- **Impact:** 불량 부품 자동 재정렬 → 공정 병목 현상 감소 및 생산 효율 향상

---

## 🎥 Demo Video

> **[여기에 시연 영상 GIF 또는 유튜브 링크 삽입]**

---
