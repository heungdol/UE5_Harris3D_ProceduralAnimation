# Harris 3D과 Procedural Animation을 활용한 등산모션 제작
2022년 1학기 소프트웨어융합캡스톤디자인 프로젝트

## 1. 개요

## 2. 활용 알고리즘 및 기술

### 2-1. Harris 3D
### 2-2. Procedural Animation
### 2-3. Megascans

## 3. 구현 및 결과 (소융캡디 최종 발표)

### 3-0. Chracter Controller
#### 3-0-1. Player Input
    - WASD: 캐릭터 이동 <br>
    - Space: 점프 / 암벽등산모드 탈출
    - 암벽접근: 암벽등산모드

### 3-1. Harris 3D Keypoint 검출

### 3-2. Procedural Animation
#### 3-2-1. 

## 4. 피드백 및 추가 구현 사항 (소융캡디 최종 발표)
    (피드백: 발표시간에 받은 사항, 수정사항: 개인적인 수정 사항)
    - 피드백: 검출시 Keypoint들이 가까이 겹쳐져 있는 부분 수정요함
    - 피드백: 자연물을 보면 Keypoint들이 제대로 검출되지 않았다고 했는데, 두 개의 임계치를 수정해서 테스트해볼 것
        - Window Size -> Vertex Ring Number
        - Harris Operator -> Fraction of the Diagonal
    - 피드백: 발표를 할때 IK에 대해 좀 더 이야기 했으면 좋았을 것
    - 수정사항: 캐릭터 컨트롤러 조작감 수정 및  Locomotion 추가
    - 수정사항: 손과 발의 각도 수정
    - 수정사항: Github 페이지 수정하기
    - 수정사항: 발표 때 Keypoint 이야기할 때 Wireframe mode로 설명했으면 더 좋았을 것
    - 수정사항: 정상에 다다를시 올라갈 수 있도록 캐릭터 컨트롤러 수정
    
## 5. 피드백 수용 및 개선

## 6. 2차 구현 및 결과
