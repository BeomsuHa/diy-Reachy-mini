# Stewart Platform Simulator (Python/Tkinter)

Stewart.js를 참고하여 만든 파이썬 Tkinter 기반 Stewart Platform 시뮬레이터입니다.

## 기능

- **XYZ 축 이동 제어**: 플랫폼의 X, Y, Z 위치를 실시간으로 조정
- **RPY 회전 제어**: Roll, Pitch, Yaw 각도로 플랫폼 회전 제어
- **실시간 역기구학 계산**: 6개 모터의 회전각을 실시간으로 계산 및 표시
- **3D 시각화**: matplotlib을 사용한 실시간 3D 플랫폼 시각화
- **파라미터 수정**: 플랫폼의 기하학적 파라미터를 동적으로 수정 가능
- **직관적인 GUI**: 슬라이더와 입력 필드를 통한 쉬운 조작

## 설치 및 실행

### 요구사항
- Python 3.6 이상
- tkinter (대부분의 Python 설치에 포함됨)
- matplotlib (3D 시각화용)
- numpy (수치 계산용)

### 설치 방법
```bash
pip install -r requirements.txt
```

### 실행 방법
```bash
python stewart_platform_simulator.py
```

## 사용법

### 1. 위치 제어 (Position Control)
- **X, Y, Z 슬라이더**: 플랫폼의 X, Y, Z 축 이동을 -50mm ~ +50mm 범위에서 조정
- **Z 축**: -30mm ~ +30mm 범위에서 조정
- 실시간으로 슬라이더를 움직이면 서보 각도와 3D 시각화가 즉시 업데이트됩니다

### 2. 회전 제어 (Rotation Control)
- **Roll, Pitch, Yaw 슬라이더**: 각 축을 중심으로 한 회전을 -30도 ~ +30도 범위에서 조정
- 실시간으로 회전 각도가 변경되면 서보 각도와 3D 시각화가 즉시 업데이트됩니다

### 3. 서보 각도 출력 (Servo Angles)
- 6개 모터의 회전각이 실시간으로 표시됩니다
- 각도는 도(degree) 단위로 표시됩니다
- 계산이 불가능한 위치/회전의 경우 "ERROR"로 표시됩니다

### 4. 3D 시각화 (3D Visualization)
- **베이스 플레이트**: 파란색 원형으로 표시, 베이스 조인트는 파란색 점으로 표시
- **플랫폼 플레이트**: 빨간색 원형으로 표시, 플랫폼 조인트는 빨간색 점으로 표시
- **다리들**: 초록색 선으로 표시, 호른 위치는 초록색 사각형으로 표시
- **좌표축**: X(빨강), Y(초록), Z(파랑) 축이 화살표로 표시
- 실시간으로 플랫폼의 움직임을 3D로 확인할 수 있습니다

### 5. 파라미터 설정 (Platform Parameters)
다음 파라미터들을 수정할 수 있습니다:

- **Base Radius**: 베이스 플레이트 반지름 (기본값: 80mm)
- **Platform Radius**: 플랫폼 플레이트 반지름 (기본값: 50mm)
- **Rod Length**: 연결 로드 길이 (기본값: 130mm)
- **Horn Length**: 서보 호른 길이 (기본값: 50mm)
- **Shaft Distance**: 베이스 샤프트 간격 (기본값: 20mm)
- **Anchor Distance**: 플랫폼 앵커 간격 (기본값: 20mm)
- **Rotation Limit**: 회전 한계 각도 (기본값: 30.0도)

파라미터를 수정한 후 "Apply Parameters" 버튼을 클릭하면 새로운 설정이 적용됩니다.

### 6. 리셋 기능
"Reset to Center" 버튼을 클릭하면 모든 위치와 회전이 0으로 초기화됩니다.

## 기술적 세부사항

### 역기구학 계산
프로그램은 Stewart Platform의 역기구학을 다음과 같이 계산합니다:

1. **쿼터니언 변환**: RPY 각도를 쿼터니언으로 변환
2. **플랫폼 조인트 회전**: 쿼터니언을 사용하여 플랫폼 조인트 위치를 회전
3. **벡터 계산**: 베이스 조인트에서 플랫폼 조인트까지의 벡터 계산
4. **서보 각도 계산**: 각 다리에 대해 서보 각도를 계산

### 수학적 공식
각 다리 i에 대해:
- `gk = l² - rod_length² + horn_length²`
- `ek = 2 × horn_length × lz`
- `fk = 2 × horn_length × (cos(βi) × lx + sin(βi) × ly)`
- `sin(αi) = (gk × ek) / (ek² + fk²) - (fk × √(1 - gk²/(ek² + fk²))) / √(ek² + fk²)`

여기서 αi는 서보 각도, βi는 모터 설치 각도입니다.

### 3D 시각화
- **matplotlib**: 3D 플롯을 위한 라이브러리
- **실시간 업데이트**: 위치나 회전이 변경될 때마다 3D 모델이 업데이트
- **인터랙티브 뷰**: 마우스로 3D 뷰를 회전하고 확대/축소 가능

## 파일 구조

```
stewart_platform_simulator.py  # 메인 프로그램
requirements.txt               # 필요한 패키지 목록
README_Python.md              # 이 파일
```

## 클래스 구조

### Quaternion
- 쿼터니언 연산을 위한 클래스
- 오일러 각도(RPY)에서 쿼터니언 변환
- 벡터 회전 기능

### StewartPlatform
- Stewart Platform의 역기구학 계산
- 플랫폼 초기화 및 파라미터 관리
- 서보 각도 계산
- 호른 위치 계산

### StewartPlatformVisualizer
- matplotlib을 사용한 3D 시각화
- 베이스/플랫폼 플레이트 그리기
- 다리와 조인트 시각화
- 실시간 업데이트

### StewartPlatformSimulator
- Tkinter 기반 GUI 인터페이스
- 실시간 제어 및 표시
- 파라미터 설정 인터페이스
- 3D 시각화 통합

## 참고사항

- 모든 각도는 라디안으로 계산되지만 도(degree)로 표시됩니다
- 서보 각도는 -90도 ~ +90도 범위로 제한됩니다
- 계산이 불가능한 위치/회전 조합의 경우 오류가 표시됩니다
- 파라미터 변경 시 플랫폼이 재초기화되므로 주의하세요
- 3D 시각화는 실시간으로 업데이트되므로 성능에 영향을 줄 수 있습니다

## 라이선스

이 프로그램은 Stewart.js 를 참고하였으며, 동일하게  MIT 라이선스를 따릅니다. 