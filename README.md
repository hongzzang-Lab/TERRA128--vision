# TERRA128 Vision Module

> RGB-D 기반 탐사로봇 지형 인식 및 BEV Costmap Navigation 모듈

TERRA128 탐사로봇 프로젝트에서 담당한 **Vision 파트**를 정리한 개인 기록용 repository

--

## Overview

탐사로봇이 토양 조사 지점까지 안전하게 이동할 수 있도록, RealSense RGB-D Camera와 IMU 데이터를 이용해 전방 지형을 분석

[주요 목표]

- 전방 지형의 3D Point Cloud 생성
- IMU Roll/Pitch 기반 카메라 자세 보정
- Bird's Eye View(BEV) Height Map 생성
- 지형의 경사, 단차, 거칠기 기반 위험도 판단
- Traversability Costmap 생성
- A* 기반 안전 경로 탐색
- RGB 화면 위 AR Overlay 형태로 위험 영역/경로 시각화

---

## Pipeline

```text
RealSense RGB-D Camera
        ↓
Point Cloud Generation
        ↓
IMU Roll/Pitch Compensation
        ↓
BEV Projection
        ↓
Terrain Feature Extraction
(Roughness / Slope / Step Height)
        ↓
Traversability Costmap
        ↓
A* Path Planning
        ↓
AR Navigation Overlay
```

---

## Features

- **RGB-D 기반 지형 인식**  
  Intel RealSense depth frame을 point cloud로 변환합니다.

- **IMU 기반 자세 보정**  
  UDP로 수신한 Roll/Pitch 값을 이용해 카메라 기울기를 보정합니다.

- **BEV Costmap 생성**  
  전방 영역을 일정 크기의 voxel grid로 나누고, 각 cell의 지형 위험도를 계산합니다.

- **지형 특징 분석**  
  각 grid cell에서 다음 특징을 계산합니다.
  - height
  - roughness
  - slope / tilt
  - step height

- **A* Path Planning**  
  단순 최단경로가 아니라, 지형 cost를 반영한 경로를 생성합니다.

- **AR Overlay Visualization**  
  BEV에서 판단한 안전/위험 영역과 경로를 원본 RGB 영상 위에 다시 투영해 시각화합니다.

---

## Repository Structure

```text
.
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   └── terra128_vision_core.py
└── docs/
    └── vision_module_summary.md
```

---

## Tech Stack

- Python
- NumPy
- OpenCV
- Intel RealSense SDK (`pyrealsense2`)
- UDP Socket Communication
- A* Path Planning

---

## Run

RealSense camera와 IMU UDP 송신부가 연결된 상태에서 실행합니다.

```bash
pip install -r requirements.txt
python src/terra128_vision_core.py
```

기본 조작:

```text
A : BEV Scan / Update
R : Reset
Q : Quit
```

---

## Current Status

- [x] RealSense depth 기반 point cloud 생성
- [x] IMU Roll/Pitch 보정
- [x] BEV height map 생성
- [x] Roughness / Slope / Step 기반 지형 분석
- [x] Traversability costmap 생성
- [x] A* path planning
- [x] AR overlay 시각화
- [ ] 실제 주행 제어 명령과 연동
- [ ] 팀 통합 시스템과 인터페이스 정리
