# Xycar Autonomous Driving (Camera + LiDAR)

카메라 기반 차선 인식과 Bird-Eye View 기반 횡단보도/정지선 인식, LiDAR 기반 장애물/터널 인식 및 제어를 통합한 Xycar 자율주행 프로젝트입니다.  
주행 상황을 우선순위 기반(state priority)으로 판단하여 **터널 → 장애물 → 횡단보도 → 정지선 → 일반주행** 순으로 처리합니다.

<img width="796" height="447" alt="image" src="https://github.com/user-attachments/assets/9c573f7e-8690-4166-9f50-e15427e1305c" />

## Demo / Features

- 차선 중심 기반 PID 주행 (직선/곡선)
- Bird-Eye View 기반 횡단보도 인식 → 5s 정지 후 재출발
- Bird-Eye View 기반 정지선 인식 → 정지 및 종료
- LiDAR 기반 장애물 인식 및 회피 주행 (좌/우 구분)
- LiDAR 기반 터널 인식 및 벽면 거리 균형 PID 주행
- Debug 모드(`Line_debug`)에서 시각화 지원

---

## System Overview

### Priority Flow (Main Loop)

주행 제어는 메인 루프에서 아래 우선순위로 판단합니다.

1. 터널 구간이면 → 터널 PID 제어 후 `continue`
2. 장애물이면 → 장애물 회피 PID 제어 후 `continue`
3. 횡단보도이면 → 정지 5s, 플래그 기록 후 `continue`
4. 정지선이면 → 정지 후 루프 종료
5. 그 외 → 차선 중심 PID 기반 일반 주행

---

## Architecture / Modules

기능별 모듈화를 위해 주요 클래스를 분리했습니다.

- `Camera`
  - ROS 이미지 구독 (`/usb_cam/image_raw`)
  - 캘리브레이션 + Bird-Eye 변환
  - 차선(`lpos/rpos`), 횡단보도/정지선 인식 정보 제공

- `Line_debug` / `Line`
  - Hough 기반 차선/선분 필터링
  - `Line_debug`: `cv2.imshow()` 기반 시각화 포함
  - `Line`: 실제 주행용(디버그 제거)
  - 상속 구조: `Line_debug -> Line`

- `LiDAR`
  - ROS LaserScan 구독 (`/scan`)
  - 터널 판단, 장애물 판단
  - 터널 주행용 좌/우 거리 추출, 장애물 거리/각도 추출

- `XycarControl`
  - PID 제어 모음
  - `PID()` 일반 주행
  - `obstacle_PID()` 장애물 회피
  - `tunnel_PID()` 터널 주행
  - `drive()` 최종 조향/속도 발행

- `config.py`
  - 카메라 내부 파라미터, 왜곡 계수
  - 캘리브레이션 ROI, 차선 검출 ROI
  - Bird-Eye 변환 좌표 (`src_pts`, `dst_pts`)
  - Bird-Eye ROI (횡단보도/정지선 구간에 따라 조정)
  - `Debug`, `crosswalk_completed` 등 상태/설정 값

---

## Perception 

### 1) Camera Calibration

Zhang’s Method 기반으로 카메라 내부 파라미터 `K` 및 왜곡 계수(distortion coefficients)를 추정했고, 아래 방식으로 보정합니다.

- Undistortion:
  - `cv2.undistort(raw, mtx, dist, None, new_mtx)`
<img width="976" height="373" alt="image" src="https://github.com/user-attachments/assets/04fa3682-8f44-4463-b8c7-c90d17ce29f0" />

이 과정을 통해 왜곡(Radial/Tangential) 및 투영 오차를 완화한 뒤 차선/표지 인식을 수행합니다.

### 2) Bird-Eye View

횡단보도/정지선을 구성하는 선분의 기하학적 형태(수직/대각/수평)를 더 뚜렷하게 만들기 위해 Perspective Transform을 적용합니다.

- `cv2.getPerspectiveTransform(src_pts, dst_pts)`
- `cv2.warpPerspective(...)`

<img width="461" height="361" alt="image" src="https://github.com/user-attachments/assets/a8fdb0ea-0dde-4621-bf19-95bfa4f3ad90" />

이후 Bird-Eye ROI 내에서 Canny + HoughLinesP로 선분을 검출하고, 각도 기반 카운팅으로 이벤트를 판단합니다.

---

## Driving / Control

### Lane Following (PID)

차선 좌/우 위치 `lpos, rpos`로 중심을 구하고, 목표 중심값을 실험적으로 보정하여 PID를 적용합니다.  
카메라 특성으로 인해 중심 목표값을 기본 `320` 대신 보정값을 사용합니다.

<img width="646" height="390" alt="image" src="https://github.com/user-attachments/assets/ccc763ce-847e-4c34-a2db-78816414075f" />
 - 차선 검출 위한 ROI
   
### Obstacle Avoidance (LiDAR)

전방 좌우 약 `100°` 범위를 확인해 threshold 내 포인트 개수로 장애물 위치를 판단합니다.

- 우측 장애물 → `right_obstacle_driving()`로 거리/각도 추정 후 PID 회피
- 좌측 장애물 → `left_obstacle_driving()` 동일 방식

<img width="313" height="341" alt="image" src="https://github.com/user-attachments/assets/b012fd88-0e72-4a85-ad33-a2a0e8cd37dc" />

장애물 회피 PID는 `d * sin(theta)` 기반으로 차량 중심축과 장애물 간 목표 거리를 유지하도록 설계했습니다.

### Tunnel Driving (LiDAR)

좌/우 벽면 거리 평균을 구해 두 거리 차이를 `error`로 두고 PID로 보정합니다.

- 목표: `d_left ≈ d_right`

<img width="357" height="396" alt="image" src="https://github.com/user-attachments/assets/325ce439-8a92-40e4-a0f6-c87e893a5881" />


## Team / Roles

| Name | Role | 
|------|------|
| 김민섭 | 주행(직선/곡선), 장애물 인식 및 제어, 터널 인식 및 제어 |
| 정지환 | 주행(직선/곡선), 장애물 인식 및 제어, 터널 인식 및 제어 |
| 최성현 | Camera Calibration, Bird-Eye View, 횡단보도 인식 및 제어 코드 구현 |
| 김민영 | 모듈화(구조 설계), 정지선 인식 및 제어 구현 |
