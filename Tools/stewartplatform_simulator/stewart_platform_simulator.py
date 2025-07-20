import tkinter as tk
from tkinter import ttk, messagebox
import math
import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class Quaternion:
    """쿼터니언 클래스 - 회전을 표현"""
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    
    @classmethod
    def from_euler(cls, roll, pitch, yaw):
        """오일러 각도(RPY)로부터 쿼터니언 생성"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return cls(w, x, y, z)
    
    def rotate_vector(self, vector):
        """벡터를 쿼터니언으로 회전"""
        # 쿼터니언 회전 공식: v' = q * v * q*
        v_quat = Quaternion(0, vector[0], vector[1], vector[2])
        q_conj = Quaternion(self.w, -self.x, -self.y, -self.z)
        
        result = self.multiply(v_quat).multiply(q_conj)
        return [result.x, result.y, result.z]
    
    def multiply(self, other):
        """쿼터니언 곱셈"""
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

class StewartPlatform:
    """Stewart Platform 역기구학 계산 클래스"""
    def __init__(self, config=None):
        if config is None:
            config = {
                'base_radius': 80,
                'platform_radius': 50,
                'rod_length': 130,
                'horn_length': 50,
                'shaft_distance': 20,
                'anchor_distance': 20,
                'rotation_limit': 30.0  # 회전 한계 추가
            }
        
        self.config = config
        self.base_joints = []
        self.platform_joints = []
        self.sin_beta = []
        self.cos_beta = []
        self.T0 = [0, 0, 0]
        self.current_translation = [0, 0, 0]
        self.current_orientation = Quaternion()
        self.horn_positions = []
        
        self._initialize_platform()
    
    def _initialize_platform(self):
        """플랫폼 초기화 - 베이스와 플랫폼 조인트 위치 계산"""
        base_radius = self.config['base_radius']
        platform_radius = self.config['platform_radius']
        shaft_distance = self.config['shaft_distance']
        anchor_distance = self.config['anchor_distance']
        
        # 6개 다리의 베이스와 플랫폼 조인트 위치 계산
        for i in range(6):
            pm = (-1) ** i
            phi_cut = (1 + i - i % 2) * math.pi / 3
            
            # 베이스 조인트 위치
            phi_b = (i + i % 2) * math.pi / 3 + pm * shaft_distance / (2 * base_radius)
            base_x = math.cos(phi_b) * base_radius
            base_y = math.sin(phi_b) * base_radius
            self.base_joints.append([base_x, base_y, 0])
            
            # 플랫폼 조인트 위치
            phi_p = phi_cut - pm * anchor_distance / (2 * platform_radius)
            platform_x = math.cos(phi_p) * platform_radius
            platform_y = math.sin(phi_p) * platform_radius
            self.platform_joints.append([platform_x, platform_y, 0])
            
            # 모터 회전각 (베타)
            motor_rotation = phi_b + ((i + 0) % 2) * math.pi + math.pi / 2
            self.sin_beta.append(math.sin(motor_rotation))
            self.cos_beta.append(math.cos(motor_rotation))
        
        # 초기 높이 계산
        self.T0[2] = math.sqrt(
            self.config['rod_length']**2 + self.config['horn_length']**2
            - (self.platform_joints[0][0] - self.base_joints[0][0])**2
            - (self.platform_joints[0][1] - self.base_joints[0][1])**2
        )
        
        # 호른 위치 초기화
        self.horn_positions = [[0, 0, 0] for _ in range(6)]
    
    def calculate_workspace_limits(self):
        """작업 공간의 한계 계산"""
        try:
            rod_length = self.config['rod_length']
            horn_length = self.config['horn_length']
            
            # 기본적인 작업 공간 한계 계산
            # 로드 길이와 호른 길이를 고려한 최대 이동 거리
            max_extension = rod_length + horn_length
            min_extension = abs(rod_length - horn_length)
            
            # 베이스와 플랫폼 반지름 차이를 고려한 XY 평면 한계
            radius_diff = abs(self.config['base_radius'] - self.config['platform_radius'])
            
            # Z축 한계 (높이) - 안전장치 추가
            z_max = self.T0[2] + max_extension
            z_min = max(0, self.T0[2] - min_extension)  # 음수 높이 방지
            
            # XY 평면 한계 (반지름) - 안전장치 추가
            xy_max = min(max_extension, self.config['base_radius'] - radius_diff)
            xy_max = max(xy_max, 10.0)  # 최소 10mm 보장
            
            # 회전 한계 (도 단위)
            rotation_max = self.config['rotation_limit']
            
            return {
                'x_range': (-xy_max, xy_max),
                'y_range': (-xy_max, xy_max),
                'z_range': (z_min - self.T0[2], z_max - self.T0[2]),
                'rotation_range': (-rotation_max, rotation_max)
            }
        except Exception as e:
            # 오류 발생 시 기본값 반환
            return {
                'x_range': (-50, 50),
                'y_range': (-50, 50),
                'z_range': (-30, 30),
                'rotation_range': (-30, 30)
            }
    
    def calculate_inverse_kinematics(self, translation, orientation):
        """역기구학 계산 - 위치와 방향으로부터 모터 각도 계산"""
        horn_length = self.config['horn_length']
        rod_length = self.config['rod_length']
        
        self.current_translation = translation
        self.current_orientation = orientation
        
        servo_angles = []
        
        for i in range(6):
            try:
                # 플랫폼 조인트를 회전
                rotated_platform = orientation.rotate_vector(self.platform_joints[i])
                
                # 플랫폼 조인트의 절대 위치
                q_x = translation[0] + rotated_platform[0]
                q_y = translation[1] + rotated_platform[1]
                q_z = translation[2] + rotated_platform[2] + self.T0[2]
                
                # 베이스에서 플랫폼 조인트까지의 벡터
                l_x = q_x - self.base_joints[i][0]
                l_y = q_y - self.base_joints[i][1]
                l_z = q_z - self.base_joints[i][2]
                
                # 역기구학 계산
                gk = l_x**2 + l_y**2 + l_z**2 - rod_length**2 + horn_length**2
                ek = 2 * horn_length * l_z
                fk = 2 * horn_length * (self.cos_beta[i] * l_x + self.sin_beta[i] * l_y)
                
                sq_sum = ek**2 + fk**2
                
                # 안전장치: sq_sum이 0이거나 너무 작은 경우
                if sq_sum < 1e-10:
                    servo_angles.append(None)
                    self.horn_positions[i] = [0, 0, 0]
                    continue
                
                # 제곱근 계산 전 안전장치
                sqrt_term = 1 - gk**2 / sq_sum
                if sqrt_term < 0:
                    # 물리적으로 불가능한 위치
                    servo_angles.append(None)
                    self.horn_positions[i] = [0, 0, 0]
                    continue
                
                sqrt1 = math.sqrt(sqrt_term)
                sqrt2 = math.sqrt(sq_sum)
                
                sin_alpha = (gk * ek) / sq_sum - (fk * sqrt1) / sqrt2
                cos_alpha = (gk * fk) / sq_sum + (ek * sqrt1) / sqrt2
                
                # 삼각함수 값 범위 체크
                if abs(sin_alpha) > 1.0:
                    # 물리적으로 불가능한 각도
                    servo_angles.append(None)
                    self.horn_positions[i] = [0, 0, 0]
                    continue
                
                # 호른 위치 계산
                self.horn_positions[i] = [
                    self.base_joints[i][0] + horn_length * cos_alpha * self.cos_beta[i],
                    self.base_joints[i][1] + horn_length * cos_alpha * self.sin_beta[i],
                    self.base_joints[i][2] + horn_length * sin_alpha
                ]
                
                # 서보 각도 계산
                servo_angle = math.asin(sin_alpha)
                
                # 각도 범위 체크 (-90도 ~ 90도)
                if -math.pi/2 <= servo_angle <= math.pi/2:
                    servo_angles.append(math.degrees(servo_angle))
                else:
                    servo_angles.append(None)
                    
            except (ValueError, ZeroDivisionError, math.DomainError) as e:
                # 수학적 오류 발생 시
                servo_angles.append(None)
                self.horn_positions[i] = [0, 0, 0]
                continue
        
        return servo_angles
    
    def get_platform_joints_world(self):
        """현재 플랫폼 조인트의 월드 좌표 반환"""
        world_joints = []
        for joint in self.platform_joints:
            rotated = self.current_orientation.rotate_vector(joint)
            world_joint = [
                self.current_translation[0] + rotated[0],
                self.current_translation[1] + rotated[1],
                self.current_translation[2] + rotated[2] + self.T0[2]
            ]
            world_joints.append(world_joint)
        return world_joints

class StewartPlatformVisualizer:
    """Stewart Platform 3D 시각화 클래스"""
    def __init__(self, platform):
        self.platform = platform
        self.fig = None
        self.ax = None
        self.canvas = None
        
    def create_visualization(self, parent_frame):
        """3D 시각화 생성"""
        # matplotlib figure 생성
        self.fig = plt.Figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Tkinter canvas에 matplotlib figure 임베드
        self.canvas = FigureCanvasTkAgg(self.fig, parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 초기 플롯
        self.update_visualization()
    
    def update_visualization(self):
        """시각화 업데이트"""
        if self.ax is None:
            return
            
        self.ax.clear()
        
        # 베이스 플레이트 그리기
        self._draw_base_plate()
        
        # 플랫폼 플레이트 그리기
        self._draw_platform_plate()
        
        # 다리들 그리기
        self._draw_legs()
        
        # 축 그리기
        self._draw_axes()
        
        # 작업 공간 한계 표시
        self._draw_workspace_limits()
        
        # 그래프 설정
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Stewart Platform 3D Visualization')
        
        # 뷰 설정
        self.ax.view_init(elev=20, azim=45)
        
        # 캔버스 업데이트
        self.canvas.draw()
    
    def _draw_base_plate(self):
        """베이스 플레이트 그리기"""
        base_radius = self.platform.config['base_radius']
        
        # 베이스 플레이트 원형
        theta = np.linspace(0, 2*np.pi, 100)
        x = base_radius * np.cos(theta)
        y = base_radius * np.sin(theta)
        z = np.zeros_like(theta)
        
        self.ax.plot(x, y, z, 'b-', linewidth=2, label='Base Plate')
        
        # 베이스 조인트들
        for i, joint in enumerate(self.platform.base_joints):
            self.ax.scatter(joint[0], joint[1], joint[2], c='blue', s=50, marker='o')
            self.ax.text(joint[0], joint[1], joint[2], f'B{i+1}', fontsize=8)
    
    def _draw_platform_plate(self):
        """플랫폼 플레이트 그리기"""
        platform_radius = self.platform.config['platform_radius']
        world_joints = self.platform.get_platform_joints_world()
        
        # 플랫폼 중심
        center = self.platform.current_translation.copy()
        center[2] += self.platform.T0[2]
        
        # 플랫폼 플레이트 원형
        theta = np.linspace(0, 2*np.pi, 100)
        x = platform_radius * np.cos(theta)
        y = platform_radius * np.sin(theta)
        z = np.zeros_like(theta)
        
        # 플랫폼 회전 적용
        for i in range(len(x)):
            point = [x[i], y[i], z[i]]
            rotated = self.platform.current_orientation.rotate_vector(point)
            x[i] = center[0] + rotated[0]
            y[i] = center[1] + rotated[1]
            z[i] = center[2] + rotated[2]
        
        self.ax.plot(x, y, z, 'r-', linewidth=2, label='Platform Plate')
        
        # 플랫폼 조인트들
        for i, joint in enumerate(world_joints):
            self.ax.scatter(joint[0], joint[1], joint[2], c='red', s=50, marker='o')
            self.ax.text(joint[0], joint[1], joint[2], f'P{i+1}', fontsize=8)
    
    def _draw_legs(self):
        """다리들 그리기"""
        world_joints = self.platform.get_platform_joints_world()
        
        for i in range(6):
            # 베이스 조인트에서 호른까지
            base_joint = self.platform.base_joints[i]
            horn_pos = self.platform.horn_positions[i]
            
            if horn_pos != [0, 0, 0]:  # 유효한 호른 위치인 경우
                self.ax.plot([base_joint[0], horn_pos[0]], 
                           [base_joint[1], horn_pos[1]], 
                           [base_joint[2], horn_pos[2]], 'g-', linewidth=3)
                
                # 호른에서 플랫폼 조인트까지
                platform_joint = world_joints[i]
                self.ax.plot([horn_pos[0], platform_joint[0]], 
                           [horn_pos[1], platform_joint[1]], 
                           [horn_pos[2], platform_joint[2]], 'g-', linewidth=3)
                
                # 호른 위치 표시
                self.ax.scatter(horn_pos[0], horn_pos[1], horn_pos[2], c='green', s=30, marker='s')
    
    def _draw_axes(self):
        """좌표축 그리기"""
        # 원점
        self.ax.scatter(0, 0, 0, c='black', s=100, marker='o')
        
        # X, Y, Z 축
        axis_length = 100
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', arrow_length_ratio=0.1, label='X')
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', arrow_length_ratio=0.1, label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', arrow_length_ratio=0.1, label='Z')
        
        self.ax.legend()
    
    def _draw_workspace_limits(self):
        """작업 공간 한계 표시"""
        limits = self.platform.calculate_workspace_limits()
        
        # XY 평면에서의 작업 공간 한계 (원형)
        xy_max = limits['x_range'][1]
        theta = np.linspace(0, 2*np.pi, 100)
        x = xy_max * np.cos(theta)
        y = xy_max * np.sin(theta)
        z_min = limits['z_range'][0] + self.platform.T0[2]
        z_max = limits['z_range'][1] + self.platform.T0[2]
        
        # 하단 원
        self.ax.plot(x, y, [z_min]*len(x), 'k--', alpha=0.3, linewidth=1, label='Workspace Limits')
        # 상단 원
        self.ax.plot(x, y, [z_max]*len(x), 'k--', alpha=0.3, linewidth=1)

class StewartPlatformGUI:
    """Stewart Platform 제어 GUI"""
    def __init__(self, root):
        self.root = root
        self.root.title("Stewart Platform Controller with 3D Visualization")
        self.root.geometry("1200x800")
        
        # Stewart Platform 인스턴스 생성
        self.platform = StewartPlatform()
        self.visualizer = StewartPlatformVisualizer(self.platform)
        
        self.create_widgets()
        self.update_servo_angles()
    
    def create_widgets(self):
        """GUI 위젯 생성"""
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 제목
        title_label = ttk.Label(main_frame, text="Stewart Platform Controller with 3D Visualization", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=6, pady=(0, 20))
        
        # 왼쪽 컨트롤 패널
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # 위치 제어 프레임
        position_frame = ttk.LabelFrame(control_frame, text="Position Control (mm)", padding="10")
        position_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # X, Y, Z 위치
        ttk.Label(position_frame, text="X:").grid(row=0, column=0, sticky=tk.W)
        self.x_var = tk.DoubleVar(value=0.0)
        self.x_scale = ttk.Scale(position_frame, from_=-50, to=50, variable=self.x_var, 
                                orient=tk.HORIZONTAL, command=self.on_position_change)
        self.x_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.x_entry = ttk.Entry(position_frame, textvariable=self.x_var, width=10)
        self.x_entry.grid(row=0, column=2, padx=(10, 0))
        
        ttk.Label(position_frame, text="Y:").grid(row=1, column=0, sticky=tk.W)
        self.y_var = tk.DoubleVar(value=0.0)
        self.y_scale = ttk.Scale(position_frame, from_=-50, to=50, variable=self.y_var, 
                                orient=tk.HORIZONTAL, command=self.on_position_change)
        self.y_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.y_entry = ttk.Entry(position_frame, textvariable=self.y_var, width=10)
        self.y_entry.grid(row=1, column=2, padx=(10, 0))
        
        ttk.Label(position_frame, text="Z:").grid(row=2, column=0, sticky=tk.W)
        self.z_var = tk.DoubleVar(value=0.0)
        self.z_scale = ttk.Scale(position_frame, from_=-30, to=30, variable=self.z_var, 
                                orient=tk.HORIZONTAL, command=self.on_position_change)
        self.z_scale.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.z_entry = ttk.Entry(position_frame, textvariable=self.z_var, width=10)
        self.z_entry.grid(row=2, column=2, padx=(10, 0))
        
        # 회전 제어 프레임
        rotation_frame = ttk.LabelFrame(control_frame, text="Rotation Control (degrees)", padding="10")
        rotation_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Roll, Pitch, Yaw 회전
        ttk.Label(rotation_frame, text="Roll:").grid(row=0, column=0, sticky=tk.W)
        self.roll_var = tk.DoubleVar(value=0.0)
        self.roll_scale = ttk.Scale(rotation_frame, from_=-30, to=30, variable=self.roll_var, 
                                   orient=tk.HORIZONTAL, command=self.on_rotation_change)
        self.roll_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.roll_entry = ttk.Entry(rotation_frame, textvariable=self.roll_var, width=10)
        self.roll_entry.grid(row=0, column=2, padx=(10, 0))
        
        ttk.Label(rotation_frame, text="Pitch:").grid(row=1, column=0, sticky=tk.W)
        self.pitch_var = tk.DoubleVar(value=0.0)
        self.pitch_scale = ttk.Scale(rotation_frame, from_=-30, to=30, variable=self.pitch_var, 
                                    orient=tk.HORIZONTAL, command=self.on_rotation_change)
        self.pitch_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.pitch_entry = ttk.Entry(rotation_frame, textvariable=self.pitch_var, width=10)
        self.pitch_entry.grid(row=1, column=2, padx=(10, 0))
        
        ttk.Label(rotation_frame, text="Yaw:").grid(row=2, column=0, sticky=tk.W)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.yaw_scale = ttk.Scale(rotation_frame, from_=-30, to=30, variable=self.yaw_var, 
                                  orient=tk.HORIZONTAL, command=self.on_rotation_change)
        self.yaw_scale.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.yaw_entry = ttk.Entry(rotation_frame, textvariable=self.yaw_var, width=10)
        self.yaw_entry.grid(row=2, column=2, padx=(10, 0))
        
        # 서보 각도 출력 프레임
        servo_frame = ttk.LabelFrame(control_frame, text="Servo Angles (degrees)", padding="10")
        servo_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 6개 서보 각도 표시
        self.servo_labels = []
        for i in range(6):
            ttk.Label(servo_frame, text=f"Servo {i+1}:").grid(row=i//3, column=(i%3)*2, sticky=tk.W, padx=(0, 10))
            label = ttk.Label(servo_frame, text="0.00°", font=("Arial", 10, "bold"))
            label.grid(row=i//3, column=(i%3)*2+1, sticky=tk.W)
            self.servo_labels.append(label)
        
        # 파라미터 설정 프레임
        param_frame = ttk.LabelFrame(control_frame, text="Platform Parameters", padding="10")
        param_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 파라미터 입력 필드들
        params = [
            ("Base Radius (mm):", "base_radius", 80),
            ("Platform Radius (mm):", "platform_radius", 50),
            ("Rod Length (mm):", "rod_length", 130),
            ("Horn Length (mm):", "horn_length", 50),
            ("Shaft Distance (mm):", "shaft_distance", 20),
            ("Anchor Distance (mm):", "anchor_distance", 20),
            ("Rotation Limit (deg):", "rotation_limit", 30.0)  # 회전 한계 파라미터 추가
        ]
        
        self.param_vars = {}
        for i, (label_text, param_name, default_value) in enumerate(params):
            ttk.Label(param_frame, text=label_text).grid(row=i//3, column=(i%3)*2, sticky=tk.W, padx=(0, 10))
            var = tk.DoubleVar(value=default_value)
            self.param_vars[param_name] = var
            entry = ttk.Entry(param_frame, textvariable=var, width=10)
            entry.grid(row=i//3, column=(i%3)*2+1, sticky=tk.W)
        
        # 파라미터 적용 버튼
        apply_button = ttk.Button(param_frame, text="Apply Parameters", command=self.apply_parameters)
        apply_button.grid(row=3, column=4, padx=(20, 0))  # 행을 3으로 변경
        
        # 작업 공간 한계 표시 프레임
        limits_frame = ttk.LabelFrame(control_frame, text="Workspace Limits", padding="10")
        limits_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.limits_labels = {}
        limits_info = [
            ("X Range:", "x_range"),
            ("Y Range:", "y_range"), 
            ("Z Range:", "z_range"),
            ("Rotation Range:", "rotation_range")
        ]
        
        for i, (label_text, limit_key) in enumerate(limits_info):
            ttk.Label(limits_frame, text=label_text).grid(row=i//2, column=(i%2)*2, sticky=tk.W, padx=(0, 10))
            label = ttk.Label(limits_frame, text="Calculating...", font=("Arial", 9))
            label.grid(row=i//2, column=(i%2)*2+1, sticky=tk.W)
            self.limits_labels[limit_key] = label
        
        # 리셋 버튼
        reset_button = ttk.Button(control_frame, text="Reset to Center", command=self.reset_to_center)
        reset_button.grid(row=5, column=0, pady=(10, 0))
        
        # 3D 시각화 프레임
        viz_frame = ttk.LabelFrame(main_frame, text="3D Visualization", padding="10")
        viz_frame.grid(row=1, column=2, columnspan=4, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 3D 시각화 생성
        self.visualizer.create_visualization(viz_frame)
        
        # 그리드 가중치 설정
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(2, weight=1)
        main_frame.columnconfigure(3, weight=1)
        main_frame.columnconfigure(4, weight=1)
        main_frame.columnconfigure(5, weight=1)
        main_frame.rowconfigure(1, weight=1)
        control_frame.columnconfigure(0, weight=1)
        position_frame.columnconfigure(1, weight=1)
        rotation_frame.columnconfigure(1, weight=1)
        servo_frame.columnconfigure(1, weight=1)
        servo_frame.columnconfigure(3, weight=1)
        servo_frame.columnconfigure(5, weight=1)
        param_frame.columnconfigure(1, weight=1)
        param_frame.columnconfigure(3, weight=1)
        param_frame.columnconfigure(5, weight=1)
        limits_frame.columnconfigure(1, weight=1)
        limits_frame.columnconfigure(3, weight=1)
        
        # 초기 작업 공간 한계 업데이트
        self.update_workspace_limits()
    
    def update_workspace_limits(self):
        """작업 공간 한계 업데이트 및 슬라이더 범위 조정"""
        try:
            limits = self.platform.calculate_workspace_limits()
            
            # 한계 정보 표시
            self.limits_labels['x_range'].config(text=f"{limits['x_range'][0]:.1f} ~ {limits['x_range'][1]:.1f} mm")
            self.limits_labels['y_range'].config(text=f"{limits['y_range'][0]:.1f} ~ {limits['y_range'][1]:.1f} mm")
            self.limits_labels['z_range'].config(text=f"{limits['z_range'][0]:.1f} ~ {limits['z_range'][1]:.1f} mm")
            self.limits_labels['rotation_range'].config(text=f"{limits['rotation_range'][0]:.1f} ~ {limits['rotation_range'][1]:.1f}°")
            
            # 슬라이더 범위 조정
            self.x_scale.config(from_=limits['x_range'][0], to=limits['x_range'][1])
            self.y_scale.config(from_=limits['y_range'][0], to=limits['y_range'][1])
            self.z_scale.config(from_=limits['z_range'][0], to=limits['z_range'][1])
            self.roll_scale.config(from_=limits['rotation_range'][0], to=limits['rotation_range'][1])
            self.pitch_scale.config(from_=limits['rotation_range'][0], to=limits['rotation_range'][1])
            self.yaw_scale.config(from_=limits['rotation_range'][0], to=limits['rotation_range'][1])
            
        except Exception as e:
            messagebox.showerror("Error", f"작업 공간 한계 계산 중 오류가 발생했습니다: {str(e)}")
    
    def on_position_change(self, event=None):
        """위치 변경 시 호출"""
        self.update_servo_angles()
    
    def on_rotation_change(self, event=None):
        """회전 변경 시 호출"""
        self.update_servo_angles()
    
    def update_servo_angles(self):
        """서보 각도 계산 및 업데이트"""
        try:
            # 현재 위치와 회전 값 가져오기
            translation = [self.x_var.get(), self.y_var.get(), self.z_var.get()]
            
            # 도를 라디안으로 변환
            roll = math.radians(self.roll_var.get())
            pitch = math.radians(self.pitch_var.get())
            yaw = math.radians(self.yaw_var.get())
            
            # 쿼터니언 생성
            orientation = Quaternion.from_euler(roll, pitch, yaw)
            
            # 역기구학 계산
            servo_angles = self.platform.calculate_inverse_kinematics(translation, orientation)
            
            # 결과 표시
            error_count = 0
            for i, angle in enumerate(servo_angles):
                if angle is not None:
                    self.servo_labels[i].config(text=f"{angle:.2f}°", foreground="black")
                else:
                    self.servo_labels[i].config(text="ERROR", foreground="red")
                    error_count += 1
            
            # 에러가 있는 경우 상태 표시
            if error_count > 0:
                # 상태바나 툴팁으로 에러 정보 표시 (선택사항)
                pass
            
            # 3D 시각화 업데이트
            self.visualizer.update_visualization()
                    
        except Exception as e:
            # 더 자세한 에러 메시지
            error_msg = f"계산 중 오류가 발생했습니다:\n{str(e)}\n\n"
            error_msg += "가능한 원인:\n"
            error_msg += "1. Z축 높이가 물리적 한계를 벗어남\n"
            error_msg += "2. 회전 각도가 너무 큼\n"
            error_msg += "3. 플랫폼 파라미터가 부적절함\n"
            error_msg += "4. 위치가 작업 공간을 벗어남"
            
            messagebox.showerror("Error", error_msg)
    
    def apply_parameters(self):
        """파라미터 적용"""
        try:
            # 새로운 설정 생성
            new_config = {}
            for param_name, var in self.param_vars.items():
                new_config[param_name] = var.get()
            
            # Stewart Platform 재초기화
            self.platform = StewartPlatform(new_config)
            self.visualizer.platform = self.platform
            
            # 작업 공간 한계 업데이트
            self.update_workspace_limits()
            
            # 서보 각도 업데이트
            self.update_servo_angles()
            
            messagebox.showinfo("Success", "파라미터가 성공적으로 적용되었습니다.")
            
        except Exception as e:
            messagebox.showerror("Error", f"파라미터 적용 중 오류가 발생했습니다: {str(e)}")
    
    def reset_to_center(self):
        """중앙 위치로 리셋"""
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        self.z_var.set(0.0)
        self.roll_var.set(0.0)
        self.pitch_var.set(0.0)
        self.yaw_var.set(0.0)
        self.update_servo_angles()

def main():
    """메인 함수"""
    root = tk.Tk()
    app = StewartPlatformGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main() 