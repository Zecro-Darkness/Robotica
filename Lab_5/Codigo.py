#!/usr/bin/env python3
"""
Control del PhantomX Pincher con interfaz gráfica completa HMI.
Versión 3 - Incluye control articular, cartesiano, visualizaciones y monitoreo en tiempo real.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from datetime import datetime
import csv

# Configuración Dynamixel
USE_XL430 = False

if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_MOVING_SPEED = 112
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')
        
        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        
        # Inicializar comunicación
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.dxl_lock = threading.Lock()
        
        # Intentar abrir puerto
        self.hardware_connected = False
        try:
            if self.port.openPort() and self.port.setBaudRate(baudrate):
                self.hardware_connected = True
                self.get_logger().info(f'Puerto {port_name} abierto correctamente')
                self.initialize_motors()
            else:
                self.get_logger().warning('No se pudo conectar al hardware. Modo simulación.')
        except:
            self.get_logger().warning('No se pudo conectar al hardware. Modo simulación.')
        
        # Joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Timer para leer hardware (20Hz)
        self.read_timer = self.create_timer(0.05, self.read_joint_positions)
        
        # Posiciones actuales, objetivo y anterior
        self.current_joint_positions = [0.0] * 5
        self.target_joint_positions = [0.0] * 5
        self.previous_joint_positions = [0.0] * 5
        
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',
            'phantomx_pincher_arm_shoulder_lift_joint',
            'phantomx_pincher_arm_elbow_flex_joint',
            'phantomx_pincher_arm_wrist_flex_joint',
            'phantomx_pincher_gripper_finger1_joint',
        ]
        
        self.joint_sign = {1: 1, 2: -1, 3: -1, 4: -1, 5: 1}
        self.emergency_stop_activated = False
        
        # Límites articulares (grados)
        self.joint_limits = {
            1: (-150, 150),
            2: (-120, 120),
            3: (-139, 139),
            4: (-98, 103),
            5: (0, 30)
        }
    
    def initialize_motors(self):
        """Configuración inicial de motores"""
        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                with self.dxl_lock:
                    self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                    write_moving_speed(self.packet, self.port, dxl_id, 100)
                    # No mover a home forzadamente, leer posición actual
                    # write_goal_position(self.packet, self.port, dxl_id, DEFAULT_GOAL)
                    
                    # Leer posición actual para sincronizar
                    if USE_XL430:
                        pos, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                    else:
                        pos, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                    
                    angle = self.dxl_to_radians(pos)
                    angle *= self.joint_sign.get(dxl_id, 1)
                    self.current_joint_positions[i] = angle
                    self.target_joint_positions[i] = angle
                    self.previous_joint_positions[i] = angle
            except:
                pass
    
    def dxl_to_radians(self, dxl_value):
        """Convierte valor Dynamixel a radianes"""
        if USE_XL430:
            center, scale = 2048.0, 2.618 / 2048.0
        else:
            center, scale = 512.0, 2.618 / 512.0
        return (dxl_value - center) * scale
    
    def radians_to_dxl(self, radians):
        """Convierte radianes a valor Dynamixel"""
        if USE_XL430:
            center, scale = 2048.0, 2048.0 / 2.618
        else:
            center, scale = 512.0, 512.0 / 2.618
        return int(radians * scale + center)
    
    def publish_joint_states(self):
        """Publica estados de articulaciones"""
        from std_msgs.msg import Header
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        self.joint_state_pub.publish(joint_state)
    
    def read_joint_positions(self):
        """Lee posiciones actuales de los motores"""
        if not self.hardware_connected or self.emergency_stop_activated:
            return

        for i, dxl_id in enumerate(self.dxl_ids):
            try:
                with self.dxl_lock:
                    if USE_XL430:
                        dxl_present_position, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                    else:
                        dxl_present_position, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                
                # Convertir a radianes
                angle = self.dxl_to_radians(dxl_present_position)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[i] = angle
            except:
                pass

    def move_motor(self, motor_id, position):
        """Mueve un motor"""
        if self.emergency_stop_activated:
            return
        
        # Actualizar objetivo (guardando el anterior)
        joint_index = self.dxl_ids.index(motor_id)
        angle = self.dxl_to_radians(position)
        angle *= self.joint_sign.get(motor_id, 1)
        
        # Solo actualizar 'anterior' si hay un cambio significativo (para evitar ruido)
        if abs(angle - self.target_joint_positions[joint_index]) > 0.001:
            self.previous_joint_positions[joint_index] = self.target_joint_positions[joint_index]
            
        self.target_joint_positions[joint_index] = angle
        
        if self.hardware_connected:
            try:
                with self.dxl_lock:
                    write_goal_position(self.packet, self.port, motor_id, position)
            except:
                pass
        else:
            # En simulación, actualizamos inmediatamente
            self.current_joint_positions[joint_index] = angle
    
    def update_speed(self, speed):
        """Actualiza velocidad de todos los motores"""
        if self.hardware_connected and not self.emergency_stop_activated:
            for motor_id in self.dxl_ids:
                try:
                    with self.dxl_lock:
                        write_moving_speed(self.packet, self.port, motor_id, speed)
                except:
                    pass
    
    def home_all_motors(self):
        """Mueve todos los motores a HOME"""
        for motor_id in self.dxl_ids:
            self.move_motor(motor_id, DEFAULT_GOAL)
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.emergency_stop_activated = True
        if self.hardware_connected:
            for dxl_id in self.dxl_ids:
                try:
                    with self.dxl_lock:
                        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                except:
                    pass

class PincherGUI:
    def __init__(self, controller):
        self.controller = controller
        self.window = tk.Tk()
        self.window.title("PhantomX Pincher - Control HMI v3")
        self.window.geometry("1200x800")
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Crear header
        self.setup_header()
        
        # Crear notebook
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Crear 5 pestañas (Tab 2 eliminada y Tab 5 renombrada)
        self.tab1 = ttk.Frame(self.notebook)
        # self.tab2 eliminada
        self.tab3 = ttk.Frame(self.notebook)
        self.tab4 = ttk.Frame(self.notebook)
        self.tab5 = ttk.Frame(self.notebook)
        self.tab6 = ttk.Frame(self.notebook)
        
        self.notebook.add(self.tab1, text='Control por Sliders')
        # self.notebook.add(self.tab2, text='Control por Valores')
        self.notebook.add(self.tab5, text='Control por Valores') # Antes Visualización Toolbox
        self.notebook.add(self.tab3, text='Control Cartesiano')
        self.notebook.add(self.tab4, text='Visualización RViz')
        self.notebook.add(self.tab6, text='Pose Cartesiana')
        
        # Configurar pestañas
        self.setup_tab1()
        # self.setup_tab2()
        self.setup_tab5() # Ahora es la segunda pestaña visible
        self.setup_tab3()
        self.setup_tab4()
        self.setup_tab6()
        self.setup_common_buttons()
        
        # Parámetros DH
        self.L1 = 0.045
        self.L2 = 0.105
        self.L3 = 0.105
        self.L4 = 0.10
        
        # Historial de poses
        self.pose_history = []
        
        # Iniciar actualizaciones
        self.update_visualization()
        self.update_cartesian_display()
    
    def setup_header(self):
        """Header con información del grupo"""
        header_frame = tk.Frame(self.window, bg="#2196F3", height=70)
        header_frame.pack(fill='x', side='top')
        header_frame.pack_propagate(False)
        
        # Título
        tk.Label(header_frame, text="PhantomX Pincher - Control HMI",
                font=("Arial", 18, "bold"), bg="#2196F3", fg="white").pack(pady=5)
        
        # Información del grupo
        info_frame = tk.Frame(header_frame, bg="#2196F3")
        info_frame.pack()
        
        tk.Label(info_frame, text="Universidad Nacional de Colombia | Robótica",
                font=("Arial", 10), bg="#2196F3", fg="white").pack(side='left', padx=20)
        
        tk.Label(info_frame, text="Grupo: [Ingrese nombres aquí]",
                font=("Arial", 10), bg="#2196F3", fg="white").pack(side='left', padx=20)
    
    def setup_tab1(self):
        """Pestaña 1: Control por Sliders (mejorada)"""
        title = tk.Label(self.tab1, text="Control por Sliders - Espacio Articular", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        motors_frame = tk.Frame(self.tab1)
        motors_frame.pack(fill='x', padx=20, pady=10)
        
        self.sliders = {}
        self.slider_value_labels = {}
        
        for i, motor_id in enumerate(self.controller.dxl_ids):
            frame = tk.Frame(motors_frame)
            frame.pack(fill='x', pady=8)
            
            # Nombre del motor
            joint_name = f'q{motor_id}'
            tk.Label(frame, text=f'{joint_name}:', font=("Arial", 10, "bold"), 
                    width=5).pack(side='left', padx=5)
            
            # Límites
            limits = self.controller.joint_limits.get(motor_id, (-150, 150))
            tk.Label(frame, text=f'{limits[0]}°', font=("Arial", 8), 
                    fg='red').pack(side='left')
            
            # Slider
            slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient=tk.HORIZONTAL,
                            length=500, resolution=1,
                            command=lambda v, m=motor_id: self.on_slider_change(m))
            slider.set(0)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            tk.Label(frame, text=f'{limits[1]}°', font=("Arial", 8), 
                    fg='red').pack(side='left')
            
            # Valor actual
            value_label = tk.Label(frame, text='0°', font=("Arial", 10, "bold"), 
                                  width=8, bg='lightblue')
            value_label.pack(side='right', padx=5)
            
            self.sliders[motor_id] = slider
            self.slider_value_labels[motor_id] = value_label
        
        # Control de velocidad
        ttk.Separator(self.tab1, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        speed_frame = tk.Frame(self.tab1)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(speed_frame, text="Velocidad:", font=("Arial", 10, "bold")).pack(anchor='w')
        self.speed_slider = tk.Scale(speed_frame, from_=0, to=MAX_SPEED, orient=tk.HORIZONTAL,
                                    command=self.on_speed_change)
        self.speed_slider.set(100)
        self.speed_slider.pack(fill='x')
        
        # Botón Home en pestaña
        tk.Button(self.tab1, text="IR A HOME", font=("Arial", 11, "bold"), 
                 bg="#2196F3", fg="white", command=self.home_all, height=2).pack(fill='x', padx=20, pady=10)
    
    def setup_tab2(self):
        """Pestaña 2: Control por Valores"""
        title = tk.Label(self.tab2, text="Control por Valores - Espacio Articular", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        motors_frame = tk.Frame(self.tab2)
        motors_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        self.entries = {}
        for motor_id in self.controller.dxl_ids:
            frame = tk.Frame(motors_frame)
            frame.pack(fill='x', pady=5)
            
            tk.Label(frame, text=f'Motor {motor_id} (q{motor_id}):', width=15).pack(side='left')
            entry = tk.Entry(frame, width=10)
            entry.insert(0, "0")
            entry.pack(side='left', padx=5)
            tk.Label(frame, text="grados").pack(side='left')
            
            btn = tk.Button(frame, text="Mover", 
                          command=lambda m=motor_id: self.move_from_entry(m))
            btn.pack(side='left', padx=10)
            
            self.entries[motor_id] = entry
    
    def setup_tab3(self):
        """Pestaña 3: Control Cartesiano (NUEVA)"""
        title = tk.Label(self.tab3, text="Control en Espacio de Tarea (Cartesiano)", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        # Advertencia
        warning = tk.Label(self.tab3, 
                          text="⚠️ Control cartesiano usa cinemática inversa. Algunas posiciones pueden no ser alcanzables.",
                          font=("Arial", 9), fg="orange", bg="lightyellow")
        warning.pack(fill='x', padx=20, pady=5)
        
        control_frame = tk.Frame(self.tab3)
        control_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        # Posición
        pos_frame = tk.LabelFrame(control_frame, text="Posición TCP", font=("Arial", 11, "bold"))
        pos_frame.pack(fill='x', pady=10)
        
        self.cart_sliders = {}
        
        for axis, limits in [('X', (-0.3, 0.3)), ('Y', (-0.3, 0.3)), ('Z', (0.0, 0.4))]:
            frame = tk.Frame(pos_frame)
            frame.pack(fill='x', pady=5, padx=10)
            
            tk.Label(frame, text=f'{axis}:', width=3).pack(side='left')
            slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient=tk.HORIZONTAL,
                            resolution=0.001, length=400,
                            command=lambda v, a=axis: self.on_cartesian_change())
            slider.set(0.15 if axis == 'X' else 0.0 if axis == 'Y' else 0.2)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            tk.Label(frame, text="m", width=3).pack(side='left')
            
            self.cart_sliders[axis] = slider
        
        # Orientación
        orient_frame = tk.LabelFrame(control_frame, text="Orientación TCP", font=("Arial", 11, "bold"))
        orient_frame.pack(fill='x', pady=10)
        
        for angle in ['Roll', 'Pitch', 'Yaw']:
            frame = tk.Frame(orient_frame)
            frame.pack(fill='x', pady=5, padx=10)
            
            tk.Label(frame, text=f'{angle}:', width=6).pack(side='left')
            slider = tk.Scale(frame, from_=-180, to=180, orient=tk.HORIZONTAL,
                            resolution=1, length=400,
                            command=lambda v, a=angle: self.on_cartesian_change())
            slider.set(0)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            tk.Label(frame, text="°", width=3).pack(side='left')
            
            self.cart_sliders[angle] = slider
        
        # Estado IK
        self.ik_status_label = tk.Label(control_frame, text="Estado: Listo", 
                                       font=("Arial", 10), fg="green")
        self.ik_status_label.pack(pady=5)
    
    def setup_tab4(self):
        """Pestaña 4: Visualización RViz"""
        import subprocess
        
        title = tk.Label(self.tab4, text="Visualización en RViz - PhantomX Pincher", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        info_frame = tk.Frame(self.tab4)
        info_frame.pack(fill='x', padx=20, pady=10)
        
        info_text = """
Esta pestaña permite visualizar el robot PhantomX Pincher en RViz.

Características:
• Modelo 3D del PhantomX Pincher
• Sincronización en tiempo real con el robot físico
• Visualización de todas las articulaciones
• Feedback visual de los movimientos
        """
        info_label = tk.Label(info_frame, text=info_text, justify=tk.LEFT, 
                             font=("Arial", 10), bg="#f0f0f0", relief="solid", padx=10, pady=10)
        info_label.pack(fill='x')
        
        controls_frame = tk.Frame(self.tab4)
        controls_frame.pack(fill='x', padx=20, pady=20)
        
        self.rviz_btn = tk.Button(controls_frame, text="LANZAR RViz", 
                                 font=("Arial", 12, "bold"), bg="#2196F3", fg="white",
                                 command=self.launch_rviz, height=2)
        self.rviz_btn.pack(fill='x', pady=5)
        
        self.stop_rviz_btn = tk.Button(controls_frame, text="DETENER RViz", 
                                      font=("Arial", 10), bg="#f44336", fg="white",
                                      command=self.stop_rviz, state=tk.DISABLED)
        self.stop_rviz_btn.pack(fill='x', pady=5)
        
        self.rviz_status_label = tk.Label(controls_frame, text="RViz no iniciado", 
                                         font=("Arial", 10), fg="red")
        self.rviz_status_label.pack(pady=10)
        
        self.rviz_process = None
    
    def setup_tab5(self):
        """Pestaña 5: Control por Valores (Antes Visualización Toolbox)"""
        title = tk.Label(self.tab5, text="Control por Valores y Visualización", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        # Frame para control
        control_frame = tk.Frame(self.tab5)
        control_frame.pack(fill='x', padx=20, pady=10)
        
        tk.Label(control_frame, text="Valores Articulares (grados):", 
                font=("Arial", 11, "bold")).pack(anchor='w')
        
        entries_frame = tk.Frame(control_frame)
        entries_frame.pack(fill='x', pady=5)
        
        self.joint_entries = {}
        joint_labels = ['q1', 'q2', 'q3', 'q4', 'q5']
        
        for i, label in enumerate(joint_labels):
            frame = tk.Frame(entries_frame)
            frame.pack(side='left', padx=5)
            
            tk.Label(frame, text=f"{label}:", font=("Arial", 9)).pack()
            entry = tk.Entry(frame, width=8, font=("Arial", 9))
            entry.insert(0, "0")
            entry.pack()
            
            self.joint_entries[label] = entry
        
        tk.Button(control_frame, text="MOVER A POSICIÓN", font=("Arial", 10, "bold"),
                 bg="#4CAF50", fg="white", command=self.move_to_joint_angles,
                 height=2).pack(fill='x', pady=10)
        
        # Poses predefinidas
        poses_frame = tk.Frame(control_frame)
        poses_frame.pack(fill='x', pady=5)
        
        tk.Label(poses_frame, text="Poses Predefinidas:", 
                font=("Arial", 10, "bold")).pack(anchor='w')
        
        poses_buttons_frame = tk.Frame(poses_frame)
        poses_buttons_frame.pack(fill='x', pady=5)
        
        self.predefined_poses = {
            "Pose 1": [0, 0, 0, 0, 0],
            "Pose 2": [25, 25, 20, -20, 0],
            "Pose 3": [-35, 35, -30, 30, 0],
            "Pose 4": [85, -20, 55, 25, 0],
            "Pose 5": [80, -35, 55, -45, 0]
        }
        
        for pose_name in self.predefined_poses.keys():
            tk.Button(poses_buttons_frame, text=pose_name, 
                     command=lambda p=pose_name: self.load_pose(p),
                     width=10).pack(side='left', padx=2)
        
        ttk.Separator(self.tab5, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # Visualización
        self.fig = Figure(figsize=(15, 5))
        self.ax1 = self.fig.add_subplot(131)
        self.ax2 = self.fig.add_subplot(132)
        self.ax3 = self.fig.add_subplot(133)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.tab5)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
    
    def setup_tab6(self):
        """Pestaña 6: Pose Cartesiana (NUEVA)"""
        title = tk.Label(self.tab6, text="Visualización Numérica - Pose Cartesiana", 
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)
        
        # Frame principal
        main_frame = tk.Frame(self.tab6)
        main_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        # Posición actual
        pos_frame = tk.LabelFrame(main_frame, text="Posición TCP Actual", 
                                 font=("Arial", 12, "bold"))
        pos_frame.pack(fill='x', pady=10)
        
        self.pos_labels = {}
        for axis in ['X', 'Y', 'Z']:
            frame = tk.Frame(pos_frame)
            frame.pack(fill='x', pady=5, padx=10)
            
            tk.Label(frame, text=f'{axis}:', font=("Arial", 11, "bold"), 
                    width=3).pack(side='left')
            label = tk.Label(frame, text='0.0000 m', font=("Arial", 11), 
                           bg='lightblue', width=15)
            label.pack(side='left', padx=10)
            
            self.pos_labels[axis] = label
        
        # Orientación actual
        orient_frame = tk.LabelFrame(main_frame, text="Orientación TCP Actual", 
                                     font=("Arial", 12, "bold"))
        orient_frame.pack(fill='x', pady=10)
        
        self.orient_labels = {}
        for angle in ['Roll', 'Pitch', 'Yaw']:
            frame = tk.Frame(orient_frame)
            frame.pack(fill='x', pady=5, padx=10)
            
            tk.Label(frame, text=f'{angle}:', font=("Arial", 11, "bold"), 
                    width=6).pack(side='left')
            label = tk.Label(frame, text='0.00°', font=("Arial", 11), 
                           bg='lightgreen', width=15)
            label.pack(side='left', padx=10)
            
            self.orient_labels[angle] = label
        
        # Valores articulares actuales
        joints_frame = tk.LabelFrame(main_frame, text="Valores Articulares Actuales", 
                                     font=("Arial", 12, "bold"))
        joints_frame.pack(fill='x', pady=10)
        
        self.joint_value_labels = {}
        for i in range(1, 6):
            frame = tk.Frame(joints_frame)
            frame.pack(fill='x', pady=3, padx=10)
            
            tk.Label(frame, text=f'q{i}:', font=("Arial", 10, "bold"), 
                    width=3).pack(side='left')
            label = tk.Label(frame, text='0.00°', font=("Arial", 10), 
                           bg='lightyellow', width=12)
            label.pack(side='left', padx=10)
            
            self.joint_value_labels[i] = label
        
        # Botón exportar eliminado
        # tk.Button(main_frame, text="Exportar Historial a CSV", 
        #          font=("Arial", 10, "bold"), bg="#FF9800", fg="white",
        #          command=self.export_to_csv).pack(pady=10)
    
    def setup_common_buttons(self):
        """Botones comunes"""
        frame = tk.Frame(self.window)
        frame.pack(fill='x', padx=20, pady=10)
        
        tk.Button(frame, text="HOME", bg="#2196F3", fg="white", font=("Arial", 10, "bold"),
                 command=self.home_all).pack(side='left', padx=10)
        
        tk.Button(frame, text="PARADA DE EMERGENCIA", bg="#f44336", fg="white",
                 font=("Arial", 10, "bold"),
                 command=self.emergency_stop).pack(side='right', padx=10)
    
    # ===== MÉTODOS DE CONTROL =====
    
    def on_slider_change(self, motor_id):
        """Callback de slider articular"""
        angle_deg = self.sliders[motor_id].get()
        angle_rad = np.deg2rad(angle_deg)
        
        # Convertir a Dynamixel
        angle_with_sign = angle_rad * self.controller.joint_sign.get(motor_id, 1)
        dxl_value = self.controller.radians_to_dxl(angle_with_sign)
        
        # Mover motor
        self.controller.move_motor(motor_id, dxl_value)
        
        # Actualizar label
        self.slider_value_labels[motor_id].config(text=f'{angle_deg}°')
    
    def on_speed_change(self, value):
        """Callback de velocidad"""
        self.controller.update_speed(int(value))
    
    def move_from_entry(self, motor_id):
        """Mover desde entry"""
        try:
            angle_deg = float(self.entries[motor_id].get())
            angle_rad = np.deg2rad(angle_deg)
            
            angle_with_sign = angle_rad * self.controller.joint_sign.get(motor_id, 1)
            dxl_value = self.controller.radians_to_dxl(angle_with_sign)
            
            self.controller.move_motor(motor_id, dxl_value)
            
            if motor_id in self.sliders:
                self.sliders[motor_id].set(angle_deg)
        except ValueError:
            messagebox.showerror("Error", "Valor inválido")
    
    def on_cartesian_change(self):
        """Callback de control cartesiano"""
        try:
            # Obtener posición deseada
            x = self.cart_sliders['X'].get()
            y = self.cart_sliders['Y'].get()
            z = self.cart_sliders['Z'].get()
            
            # Obtener Pitch deseado (convertir a radianes)
            pitch_deg = self.cart_sliders['Pitch'].get()
            pitch_rad = np.deg2rad(pitch_deg)
            
            target_pos = np.array([x, y, z])
            
            # Cinemática inversa Geométrica
            # Pasamos el pitch deseado
            solution_q, success = self.inverse_kinematics(target_pos, target_pitch=pitch_rad)
            
            if success:
                # Mover a la solución
                for i, motor_id in enumerate(self.controller.dxl_ids[:4]):
                    # Verificar límites antes de mover
                    min_deg, max_deg = self.controller.joint_limits.get(motor_id, (-150, 150))
                    angle_deg = np.rad2deg(solution_q[i])
                    
                    if not (min_deg <= angle_deg <= max_deg):
                        self.ik_status_label.config(text=f"Límite articular excedido en ID {motor_id}", fg="orange")
                        return

                    angle_with_sign = solution_q[i] * self.controller.joint_sign.get(motor_id, 1)
                    dxl_value = self.controller.radians_to_dxl(angle_with_sign)
                    self.controller.move_motor(motor_id, dxl_value)
                    
                    # Actualizar sliders y entries de articulaciones
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(angle_deg)
                    if motor_id in self.entries:
                        self.entries[motor_id].delete(0, tk.END)
                        self.entries[motor_id].insert(0, f"{angle_deg:.2f}")
            
                self.ik_status_label.config(text="Estado: IK convergió ✓", fg="green")
            else:
                self.ik_status_label.config(text="Estado: Fuera de espacio de trabajo ✗", fg="red")
        
        except Exception as e:
            self.ik_status_label.config(text=f"Error: {str(e)}", fg="red")
    
    def load_pose(self, pose_name):
        """Carga pose predefinida"""
        pose = self.predefined_poses[pose_name]
        joint_labels = ['q1', 'q2', 'q3', 'q4', 'q5']
        
        for i, label in enumerate(joint_labels):
            self.joint_entries[label].delete(0, tk.END)
            self.joint_entries[label].insert(0, str(pose[i]))
    
    def move_to_joint_angles(self):
        """Mueve a ángulos especificados"""
        try:
            joint_labels = ['q1', 'q2', 'q3', 'q4', 'q5']
            angles_deg = []
            
            for label in joint_labels:
                value = float(self.joint_entries[label].get())
                angles_deg.append(value)
            
            angles_rad = [np.deg2rad(angle) for angle in angles_deg]
            
            if USE_XL430:
                center, scale = 2048, 2048 / 2.618
            else:
                center, scale = 512, 512 / 2.618
            
            for i, motor_id in enumerate(self.controller.dxl_ids):
                if i < len(angles_rad):
                    angle_with_sign = angles_rad[i] * self.controller.joint_sign.get(motor_id, 1)
                    dxl_value = int(angle_with_sign * scale + center)
                    dxl_value = max(0, min(1023 if not USE_XL430 else 4095, dxl_value))
                    
                    self.controller.move_motor(motor_id, dxl_value)
                    
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(angles_deg[i])
            
            messagebox.showinfo("Éxito", f"Robot movido a: {angles_deg}")
            
        except ValueError:
            messagebox.showerror("Error", "Valores inválidos")
    
    def home_all(self):
        """Mover a HOME"""
        self.controller.home_all_motors()
        for motor_id in self.sliders:
            self.sliders[motor_id].set(0)
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.controller.emergency_stop()
        messagebox.showwarning("Emergencia", "Parada de emergencia activada")
    
    def launch_rviz(self):
        """Lanza RViz"""
        import subprocess
        try:
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]
            
            def run_rviz():
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                self.window.after(0, self.on_rviz_closed)
            
            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()
            
            self.rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
            self.stop_rviz_btn.config(state=tk.NORMAL, bg="#f44336")
            self.rviz_status_label.config(text="RViz ejecutándose", fg="green")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo lanzar RViz: {str(e)}")
    
    def stop_rviz(self):
        """Detiene RViz"""
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        self.on_rviz_closed()
    
    def on_rviz_closed(self):
        """Callback cuando RViz se cierra"""
        self.rviz_btn.config(state=tk.NORMAL, bg="#2196F3")
        self.stop_rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
        self.rviz_status_label.config(text="RViz no iniciado", fg="red")
    
    # Método export_to_csv eliminado
    
    # ===== CINEMÁTICA =====
    
    def dh_transform(self, theta, d, a, alpha):
        """Transformación DH"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, q):
        """Cinemática directa"""
        T1 = self.dh_transform(q[0], self.L1, 0, -np.pi/2)
        T2 = self.dh_transform(q[1] - np.pi/2, 0, self.L2, 0)
        T3 = self.dh_transform(q[2], 0, self.L3, 0)
        T4 = self.dh_transform(q[3], 0, self.L4, 0)
        
        T01 = T1
        T02 = T01 @ T2
        T03 = T02 @ T3
        T04 = T03 @ T4
        
        return T04
    
    def compute_jacobian(self, q, delta=0.0001):
        """Calcula Jacobiano numérico"""
        J = np.zeros((3, 4))
        
        T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]
        
        for i in range(4):
            q_delta = q.copy()
            q_delta[i] += delta
            T_delta = self.forward_kinematics(q_delta)
            p_delta = T_delta[:3, 3]
            
            J[:, i] = (p_delta - p0) / delta
        
        return J
    
    def inverse_kinematics(self, target_pos, target_pitch=0.0):
        """Cinemática Inversa Geométrica para PhantomX Pincher (4DOF)"""
        x, y, z = target_pos
        
        # 1. Base (q1)
        q1 = np.arctan2(y, x)
        
        # 2. Proyección Planar
        # Distancia radial al objetivo
        r = np.sqrt(x**2 + y**2)
        # Altura relativa al hombro (L1 offset)
        z_eff = z - self.L1
        
        # 3. Posición de la Muñeca (Wrist Center)
        # Retrocedemos desde el efector final usando L4 y el Pitch deseado
        # Pitch es la suma de q2 + q3 + q4 (en el plano)
        # Nota: Ajustar según la definición de 0 grados del robot
        
        rw = r - self.L4 * np.cos(target_pitch)
        zw = z_eff - self.L4 * np.sin(target_pitch)
        
        # 4. Resolución 2-Link (L2, L3) para llegar a (rw, zw)
        # Ley de Cosenos
        D = (rw**2 + zw**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        
        # Verificar alcanzabilidad
        if abs(D) > 1.0:
            return None, False # Fuera del espacio de trabajo
            
        # Codo abajo (Elbow Down) es la configuración típica
        q3 = np.arccos(D) 
        # Para codo arriba sería -arccos(D)
        
        # Calcular q2
        # alpha = atan2(zw, rw)
        # beta = atan2(L3*sin(q3), L2 + L3*cos(q3))
        # q2 = alpha - beta
        alpha = np.arctan2(zw, rw)
        beta = np.arctan2(self.L3 * np.sin(q3), self.L2 + self.L3 * np.cos(q3))
        q2 = alpha - beta
        
        # 5. Calcular q4
        # q2 + q3 + q4 = pitch
        # Pero ojo con los offsets de DH.
        # En FK: T2 tiene q2 - pi/2. 
        # Significa que el q2 geométrico (0 horizontal) corresponde a q2_dh = q2_geom + pi/2
        # Vamos a calcular los ángulos geométricos puros y luego ajustar a DH/Motores
        
        # Ajuste de q2 para coincidir con DH (donde -90 es vertical arriba)
        # Si q2_geom=0 es horizontal, y q2_dh=0 es vertical...
        # Espera, revisemos forward_kinematics:
        # T2 = dh(q[1] - pi/2, ...)
        # Si q[1]=0 (Home), T2 rota -90. El brazo apunta arriba.
        # Si q[1]=90, T2 rota 0. El brazo apunta horizontal.
        # Mi q2 geométrico (alpha-beta) es 0 cuando apunta al horizonte?
        # alpha = atan2(zw, rw). Si zw=0, rw>0 -> alpha=0.
        # beta ~ 0. q2 ~ 0.
        # Entonces q2_geom 0 es horizontal.
        # Para que el robot esté horizontal, q[1] debe ser 0 (según Home) o 90?
        # En el robot real, Home (0,0,0,0) es forma de L (Arriba, Arriba, Arriba).
        # Entonces q2_geom=0 (horizontal) corresponde a q[1] = 0? No.
        # Si q[1]=0, el brazo está vertical (L).
        # Si q[1]=pi/2, el brazo está horizontal?
        # T2(q-pi/2). Si q=0 -> rot(-90). Vertical. Correcto.
        # Si q=pi/2 -> rot(0). Horizontal. Correcto.
        # Entonces q[1] (motor) = q2_geom + pi/2.
        
        q2_motor = q2 + np.pi/2
        
        # q3: En DH es directo. Si q3=0, L3 sigue a L2.
        # En geométrico, q3 es ángulo entre L2 y L3.
        # Si están alineados, q3=0.
        # Entonces q3_motor = q3.
        q3_motor = q3
        
        # q4: q2_geom + q3 + q4_geom = pitch
        # q4_geom = pitch - q2_geom - q3
        # En DH, T4 es directo.
        # q4_motor = q4_geom.
        q4_motor = target_pitch - q2 - q3
        
        return np.array([q1, q2_motor, q3_motor, q4_motor, 0.0]), True
    
    def rotation_matrix_to_euler(self, R):
        """Convierte matriz de rotación a ángulos de Euler (Roll-Pitch-Yaw)"""
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        
        if sy > 1e-6:
            roll = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = np.arctan2(R[1,0], R[0,0])
        else:
            roll = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = 0
            
        # Debug: Imprimir si el roll está fijo
        # print(f"R: \n{R}")
        # print(f"Roll: {np.rad2deg(roll)}")
        
        return roll, pitch, yaw
    
    # ===== ACTUALIZACIÓN EN TIEMPO REAL =====
    
    def update_visualization(self):
        """Actualiza visualización Toolbox"""
        # Calcular cinemática para posición ACTUAL
        q_current = self.controller.current_joint_positions[:4]
        transforms_current = self.calculate_transforms(q_current)
        pos_current = np.array([[T[0,3], T[1,3], T[2,3]] for T in transforms_current])
        
        # Calcular cinemática para posición ANTERIOR
        q_prev = self.controller.previous_joint_positions[:4]
        transforms_prev = self.calculate_transforms(q_prev)
        pos_prev = np.array([[T[0,3], T[1,3], T[2,3]] for T in transforms_prev])
        
        # Vista superior
        self.ax1.clear()
        # Plot Previous (Ghost)
        self.ax1.plot(pos_prev[:,0], pos_prev[:,1], 'o--', linewidth=2, color='red', alpha=0.8, label='Anterior')
        # Plot Current
        self.ax1.plot(pos_current[:,0], pos_current[:,1], 'o-', linewidth=3, color='blue', label='Actual')
        
        self.ax1.plot([0], [0], 'ro', markersize=10)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_title('Vista Superior (X-Y)')
        self.ax1.grid(True)
        self.ax1.set_xlim([-0.3, 0.3])
        self.ax1.set_ylim([-0.3, 0.3])
        self.ax1.set_aspect('equal')
        self.ax1.legend()
        
        # Vista lateral
        self.ax2.clear()
        # Plot Previous (Ghost)
        self.ax2.plot(pos_prev[:,0], pos_prev[:,2], 'o--', linewidth=2, color='red', alpha=0.8, label='Anterior')
        # Plot Current
        self.ax2.plot(pos_current[:,0], pos_current[:,2], 'o-', linewidth=3, color='blue', label='Actual')
        
        self.ax2.plot([0], [0], 'ro', markersize=10)
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Z (m)')
        self.ax2.set_title('Vista Lateral (X-Z)')
        self.ax2.grid(True)
        self.ax2.set_xlim([-0.4, 0.4])
        self.ax2.set_ylim([0, 0.4])
        self.ax2.legend()
    
        # Vista Frontal (Y-Z)
        self.ax3.clear()
        # Plot Previous (Ghost)
        self.ax3.plot(pos_prev[:,1], pos_prev[:,2], 'o--', linewidth=2, color='red', alpha=0.8, label='Anterior')
        # Plot Current
        self.ax3.plot(pos_current[:,1], pos_current[:,2], 'o-', linewidth=3, color='blue', label='Actual')
        
        self.ax3.plot([0], [0], 'ro', markersize=10)
        self.ax3.set_xlabel('Y (m)')
        self.ax3.set_ylabel('Z (m)')
        self.ax3.set_title('Vista Frontal (Y-Z)')
        self.ax3.grid(True)
        self.ax3.set_xlim([-0.3, 0.3])
        self.ax3.set_ylim([0, 0.4])
        self.ax3.legend()
        
        self.canvas.draw()
        self.window.after(100, self.update_visualization)
        
    def calculate_transforms(self, q):
        """Calcula todas las transformaciones para una configuración q"""
        T1 = self.dh_transform(q[0], self.L1, 0, -np.pi/2)
        T2 = self.dh_transform(q[1] - np.pi/2, 0, self.L2, 0)
        T3 = self.dh_transform(q[2], 0, self.L3, 0)
        T4 = self.dh_transform(q[3], 0, self.L4, 0)
        
        T01 = T1
        T02 = T01 @ T2
        T03 = T02 @ T3
        T04 = T03 @ T4
        
        return [np.eye(4), T01, T02, T03, T04]
    
    def update_cartesian_display(self):
        """Actualiza display de pose cartesiana y sliders cartesianos"""
        q = self.controller.current_joint_positions[:4]
        T = self.forward_kinematics(q)
        
        # Posición
        pos = T[:3, 3]
        self.pos_labels['X'].config(text=f'{pos[0]:.4f} m')
        self.pos_labels['Y'].config(text=f'{pos[1]:.4f} m')
        self.pos_labels['Z'].config(text=f'{pos[2]:.4f} m')
        
        # Actualizar sliders cartesianos (si no se están moviendo activamente)
        # Nota: scale.set() no dispara el callback command en Tkinter
        try:
            self.cart_sliders['X'].set(pos[0])
            self.cart_sliders['Y'].set(pos[1])
            self.cart_sliders['Z'].set(pos[2])
        except:
            pass
        
        # Orientación
        R = T[:3, :3]
        roll, pitch, yaw = self.rotation_matrix_to_euler(R)
        self.orient_labels['Roll'].config(text=f'{np.rad2deg(roll):.2f}°')
        self.orient_labels['Pitch'].config(text=f'{np.rad2deg(pitch):.2f}°')
        self.orient_labels['Yaw'].config(text=f'{np.rad2deg(yaw):.2f}°')
        
        try:
            self.cart_sliders['Roll'].set(np.rad2deg(roll))
            self.cart_sliders['Pitch'].set(np.rad2deg(pitch))
            self.cart_sliders['Yaw'].set(np.rad2deg(yaw))
        except:
            pass
        
        # Valores articulares
        for i in range(1, 6):
            angle_rad = self.controller.current_joint_positions[i-1]
            angle_deg = np.rad2deg(angle_rad)
            self.joint_value_labels[i].config(text=f'{angle_deg:.2f}°')
        
        # Guardar en historial
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        pose_data = [timestamp, pos[0], pos[1], pos[2], 
                    np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)]
        pose_data.extend([np.rad2deg(self.controller.current_joint_positions[i]) for i in range(5)])
        
        self.pose_history.append(pose_data)
        if len(self.pose_history) > 100:  # Mantener solo últimas 100
            self.pose_history.pop(0)
        
        self.window.after(100, self.update_cartesian_display)
    
    def on_close(self):
        """Cerrar ventana"""
        self.window.destroy()
        rclpy.shutdown()
    
    def run(self):
        """Ejecutar GUI"""
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    controller = PincherController()
    
    def spin_ros():
        rclpy.spin(controller)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    gui = PincherGUI(controller)
    gui.run()

if __name__ == '__main__':
    main()
