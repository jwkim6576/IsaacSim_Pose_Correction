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


이 시스템은 크게 **인지(Perception)**, **판단(Decision)**, **제어(Control)** 3단계로 구성됩니다.

1.  **Input:** RealSense Depth Camera를 통해 RGB 및 Depth 데이터 수집
2.  **3D Pose Estimation:** YOLOv8-OBB로 객체의 2D 좌표와 기울기($\theta$)를 검출하고, Depth 정보를 결합해 3D 공간 좌표(X, Y, Z)로 변환
3.  **Robot Control:** 보정이 필요한 각도(Threshold 초과 시)가 감지되면 로봇이 해당 좌표로 이동하여 부품을 정렬

<br>

## 💻 Tech Stack

| Category | Technology |
| :---: | :--- |
| **Simulation** | ![IsaacSim](https://img.shields.io/badge/NVIDIA-Isaac_Sim-76B900?style=flat-square&logo=nvidia) ![Omniverse](https://img.shields.io/badge/NVIDIA-Omniverse-76B900?style=flat-square&logo=nvidia) |
| **OS / Middleware** | ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu) ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros) |
| **AI / Vision** | ![YOLOv8](https://img.shields.io/badge/YOLO-v8_OBB-00FFFF?style=flat-square) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv) ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=flat-square&logo=pytorch) |
| **Hardware** | UR10 / Doosan M0609, Intel RealSense D455 |
| **Language** | ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python) ![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=c%2B%2B) |

<br>

## 🚀 Key Features & Logic

### 1. Oriented Bounding Box (OBB) Detection
기존의 수평적인 Bounding Box(AABB)는 회전된 물체의 정확한 각도를 알 수 없는 한계가 있었습니다. 이를 극복하기 위해 **YOLOv8-OBB** 모델을 도입하여 물체의 **Heading Angle(Yaw)** 값을 실시간으로 추론했습니다.

> **[여기에 PPT 17페이지의 YOLO 탐지 결과(초록색 박스 쳐진 것) 이미지를 넣으세요]**

### 2. 3D Coordinate Conversion (Deprojection)
2D 이미지 상의 픽셀 좌표 $(u, v)$를 3D 로봇 좌표계 $(x, y, z)$로 변환하기 위해 핀홀 카메라 모델을 적용했습니다.

$$
X = (u - c_x) \times Z / f_x \\
Y = (v - c_y) \times Z / f_y
$$

* **$Z$**: Depth Map에서 추출한 심도 값
* **$f_x, f_y$**: 카메라 초점 거리 (Focal Length)
* **$c_x, c_y$**: 주점 (Principal Point)

### 3. Digital Twin Simulation
물리 엔진이 적용된 Isaac Sim 환경에서 컨베이어 벨트의 마찰력과 로봇의 동역학을 시뮬레이션하여, 실제 현장 도입 시 발생할 수 있는 시행착오를 최소화했습니다.

> **[여기에 PPT 10페이지나 11페이지의 시뮬레이션 환경 캡처를 넣으세요]**

<br>

## 📊 Project Results

* [cite_start]**Detection Accuracy:** mAP50-95 기준 **90% 이상** 달성 [cite: 140]
* [cite_start]**Pose Estimation Error:** 평균 오차 **5도 내외**로 정밀 보정 성공 [cite: 382]
* **Impact:** 불량 부품의 자동 재정렬을 통해 공정 병목 현상 해소 및 생산 효율 증대 기대

<br>

## 🎥 Demo Video

> **[여기에 시연 영상 GIF나 유튜브 링크를 넣으면 완벽합니다!]**
