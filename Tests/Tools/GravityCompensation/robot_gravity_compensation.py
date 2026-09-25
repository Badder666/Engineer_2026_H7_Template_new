# -*- coding: utf-8 -*-
# Project: ArmGravComp-Qt (6-DOF Robotic Arm Gravity Compensation & Visualization)
# File: robot_gravity_compensation.py
#
# Copyright (c) 2025, Chen XingYu. All rights reserved.
#
# License: Non-Commercial Use Only / 仅限非商业使用
# -----------------------------------------------------------------------------
# 本代码及其衍生作品仅允许用于个人学习、学术研究与教学等非商业场景。
# 严禁任何形式的商业使用，包括但不限于：出售、付费服务、SaaS/在线服务、
# 广告变现、集成到商业产品或用于商业咨询/竞赛/投标等。如需商业授权，请
# 先行获得版权所有者书面许可并签署授权协议。
#
# 允许的非商业使用条件：
# 1) 保留本版权与许可声明；2) 在衍生作品/发表物中进行署名（Lu Yaoheng）并
# 标明来源仓库；3) 不得移除或修改本段声明。
#
# 免责声明：本代码按“现状”提供，不含任何明示或默示担保。作者不对因使用本
# 代码产生的任何直接或间接损失承担责任。使用者需自行评估并承担风险。
#
# English Summary:
# This code is provided for personal, academic, and research purposes only.
# Any commercial use (sale, paid service, SaaS, ad-monetization, integration
# into commercial products, consultancy, competitions, bids, etc.) is strictly
# prohibited without prior written permission from the copyright holder.
# Keep this notice intact and provide proper attribution in derived works.
# Provided "as is" without warranty of any kind. Use at your own risk.
#
# Contact / 商务与授权联系: <cdssywc@163.com>


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QSlider, QGroupBox, QTableWidget,
    QTableWidgetItem, QSplitter, QTextEdit, QTabWidget, QCheckBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QCoreApplication
from PyQt5.QtGui import QFont

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# ================= 机械臂模型 =================
class RobotModel:
    def __init__(self):
        # ---- 改良DH参数（示例，可替换为实机）----
        self.alpha = np.array([0.0, 90.0, 0.0, 90.0, 90.0, -90.0]) * np.pi/180
        #self.a     = np.array([0.0, 0.0, 0.3073, 0.0, 0.0, 0.0])
        #self.d     = np.array([0.0, 0.0, 0.0, 0.2465, 0.0, 0.081])
        self.a     = np.array([0.0, 0.0, 0.322, 0.0, 0.0, 0.0])
        # 关节偏置 (沿 z 方向)
        #   d[0]=0.127 : 基座到 J2 (肩) 的高度
        #   d[5]=0.081 : 末端法兰到工具中心
        self.d     = np.array([0.127, 0.0, 0.0, 0.266, 0.0, 0.081])
        self.theta_offset = np.array([90.0, 0.0, 90.0, 0.0, 0.0, 0.0]) * np.pi/180

        # 质心（关键：第 i 个质心在"父坐标系 i-1"中定义；关节1的父系=世界系）
        self.com_local = np.array([
            [ 0.0,      0.0,      0.0    ],  # 关节1质心（父系=世界）
            [ 0.0,      0.0,      0.0    ],  # 关节2质心（父系=关节1）
            [ 0.204,  0,  0 ],  # 关节3质心（父系=关节2）
            [-0,   -0.122,  -0 ],  # 关节4质心（父系=关节3）
            [ 0.0000,   0,  -0.0147 ],  # 关节5质心（父系=关节4）
            [ 0.0,      0.0767,   0 ],  # 关节6质心（父系=关节5）
        ], dtype=float) 
        # ---- 添加动力学参数 ----
        # 连杆质量 (kg) - 与 GravityCompensation.c 中的参数保持一致
        self.masses = np.array([3.092, 2.933, 0.955, 0.631, 0.439, 0.746], dtype=float)
        # 重力向量（世界坐标系）
        self.gravity = np.array([0, 0, -9.79], dtype=float)

        # 当前关节角
        self.current_theta = np.zeros(6, dtype=float)

        # 预设姿态
        self.test_poses = {
            "零位 [0,0,0,0,0,0]": [0, 0, 0, 0, 0, 0],
            "反向大臂180° [0,-π,0,0,0,0]": [0, np.pi, 0, 0, 0, 0],
            "大臂抬起45°": [0, np.pi/4, 0, 0, 0, 0],
            "大臂水平，小臂上垂": [0, 0, np.pi/2, 0, 0, 0],
            "复杂姿态": [0, np.pi/3, -np.pi/6, -np.pi/4, np.pi/6, 0],
            "收纳位置": [0, np.pi, np.pi/1.2, 0, -np.pi/2, 0]
        }

    # ---------- Kinematics ----------
    def dh_transform(self, a, alpha, d, theta):
        # 经典DH矩阵（与你原先使用一致）
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct,    -st,    0,      a],
            [st*ca, ct*ca,  -sa,    -d*sa],
            [st*sa, ct*sa,  ca,     d*ca],
            [0,     0,      0,      1]
        ])

    def forward_kinematics(self, theta):
        Ts = [np.eye(4)]
        T = np.eye(4)
        for i in range(6):
            T_i = self.dh_transform(self.a[i], self.alpha[i], self.d[i], theta[i] + self.theta_offset[i])
            T = T @ T_i
            Ts.append(T.copy())
        return Ts

    def get_joint_positions(self, theta):
        Ts = self.forward_kinematics(theta)
        pts = np.array([T[:3, 3] for T in Ts])
        return pts, Ts

    # --- CoM（父系→世界） ---
    def get_com_positions(self, theta):
        _, Ts = self.get_joint_positions(theta)
        coms = [Ts[i][:3, :3] @ self.com_local[i] + Ts[i][:3, 3] for i in range(6)]
        return np.array(coms)

    # --- CoM 不变量自检 ---
    def check_com_invariance(self, theta, atol=1e-9):
        Ts = self.forward_kinematics(theta)
        errs = []
        for j in range(6):
            R_parent = Ts[j][:3, :3]
            o_parent = Ts[j][:3, 3]
            com_w = R_parent @ self.com_local[j] + o_parent
            back_local = R_parent.T @ (com_w - o_parent)
            errs.append(np.linalg.norm(back_local - self.com_local[j]))
        return float(np.max(errs)), np.array(errs)

    # ============ 重力补偿计算模块 ============
    def compute_gravity_compensation(self, theta):
        """
        计算6轴机械臂的重力补偿力矩
        基于 τ_g = Σ_j ( (r_i - o_j) × (m_i * g) ) · z_j
        注意：为修复显示错位，结果在末尾整体左移一位（np.roll(-1)）
        """
        gravity_torques = np.zeros(6)

        # 所有关节/末端的变换与各连杆质心位置（世界系）
        Ts = self.forward_kinematics(theta)          # Ts[0]为基座，Ts[1]为关节1坐标系...Ts[6]为末端
        com_positions = self.get_com_positions(theta)

        # 遍历每个连杆的重力对各上游关节的贡献
        for link_idx in range(6):
            com_world  = com_positions[link_idx]
            F_gravity  = self.masses[link_idx] * self.gravity

            # 只有 joint_idx <= link_idx 的关节会对该连杆产生力矩
            for joint_idx in range(link_idx + 1):
                if joint_idx == 0:
                    # 关节1对应 z_0、o_0（即世界系Z轴与原点）
                    z_axis = np.array([0.0, 0.0, 1.0])
                    joint_origin = np.zeros(3)
                else:
                    # 其余关节使用其坐标系的Z轴与原点
                    z_axis = Ts[joint_idx][:3, 2]
                    joint_origin = Ts[joint_idx][:3, 3]

                r_vec = com_world - joint_origin
                torque_contribution = -np.dot(np.cross(r_vec, F_gravity), z_axis)
                gravity_torques[joint_idx] += torque_contribution

        # ★ 关键修复：结果整体左移一位，使 [τ1..τ6] 与 [关节1..6] 对齐
        gravity_torques = np.roll(gravity_torques, -1)
        return gravity_torques

    
    def compute_jacobian_velocity(self, theta):
        """
        计算速度雅可比矩阵（用于验证重力补偿计算）
        返回6x6矩阵，每列对应一个关节的贡献
        """
        Ts = self.forward_kinematics(theta)
        ee_pos = Ts[-1][:3, 3]  # 末端位置
        
        J_v = np.zeros((3, 6))  # 线性速度雅可比（3x6）
        
        for i in range(6):
            if i == 0:
                # 基座关节
                z_i = np.array([0, 0, 1])
                o_i = np.zeros(3)
            else:
                z_i = Ts[i][:3, 2]  # 关节i的Z轴
                o_i = Ts[i][:3, 3]  # 关节i的原点
            
            # 线性速度雅可比列：z_i × (p_ee - p_i)
            J_v[:, i] = np.cross(z_i, ee_pos - o_i)
        
        return J_v
    
    def get_dynamic_info(self, theta):
        """
        获取动力学相关信息用于显示
        """
        gravity_torques = self.compute_gravity_compensation(theta)
        com_positions = self.get_com_positions(theta)
        
        # 计算总重心
        total_mass = np.sum(self.masses)
        total_com = np.sum(self.masses.reshape(-1, 1) * com_positions, axis=0) / total_mass
        
        # 计算重力势能
        potential_energy = np.sum(self.masses * com_positions[:, 2]) * (-self.gravity[2])
        
        info = {
            'gravity_torques': gravity_torques,
            'total_mass': total_mass,
            'total_com': total_com,
            'potential_energy': potential_energy,
            'individual_masses': self.masses,
            'individual_coms': com_positions
        }
        
        return info

    # --- 欧拉角 ---
    @staticmethod
    def rot_to_rpy_zyx(R):
        eps = 1e-9
        if abs(R[2,0]) < 1 - eps:
            yaw   = np.arctan2(R[1,0], R[0,0])
            pitch = np.arcsin(-R[2,0])
            roll  = np.arctan2(R[2,1], R[2,2])
        else:
            yaw = np.arctan2(-R[0,1], R[1,1])
            pitch = np.pi/2 if R[2,0] <= -1 else -np.pi/2
            roll = 0.0
        return np.array([yaw, pitch, roll])

# ================= 画布（3D，仅固定0.8m范围 + 正方体显示） =================
class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(12, 8), facecolor='#0f0f0f')
        super().__init__(self.fig)
        self.setParent(parent)
        self.robot_model = RobotModel()

        self.show_joint_frames = True
        self.show_world_frame  = True
        self.show_ee_frame     = True
        self.show_com          = True   # 质心开关
        self.show_total_com    = True   # 总质心开关

        # 连杆配色（可按需调整）
        self.link_colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']

        # —— 固定坐标范围：0.8 m —— #
        self.fixed_xy_range = 0.8   # X/Y 方向总宽度
        self.fixed_z_range  = 0.8   # Z 方向高度（基座到顶）
        self.frame_length   = 0.12  # 坐标轴箭头长度（常量，不随缩放）

        self._build_axes()
        self.update_plot()

    def _build_axes(self):
        self.fig.clear()
        self.ax_3d = self.fig.add_subplot(111, projection='3d', facecolor='#1e1e1e')

        ax = self.ax_3d
        ax.tick_params(colors='white')
        for spine in ax.spines.values():
            spine.set_color('#666666')
        ax.title.set_color('white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.zaxis.label.set_color('white')
        self.fig.suptitle('机械臂结构与坐标系（固定0.8 m范围 | X红 / Y绿 / Z蓝）', fontsize=16, color='white', fontweight='bold')

    def set_frame_toggles(self, show_joint, show_world, show_ee):
        self.show_joint_frames = show_joint
        self.show_world_frame  = show_world
        self.show_ee_frame     = show_ee
        self.update_plot()

    def set_com_toggle(self, show):
        self.show_com = show
        self.update_plot()
    
    def set_total_com_toggle(self, show):
        self.show_total_com = show
        self.update_plot()

    def update_plot(self):
        theta = self.robot_model.current_theta
        joint_positions, Ts = self.robot_model.get_joint_positions(theta)
        com_positions       = self.robot_model.get_com_positions(theta)

        self.ax_3d.cla()
        self._plot_3d_robot(joint_positions, com_positions, Ts)
        self.draw()

    def _plot_3d_robot(self, joint_positions, com_positions, Ts):
        ax = self.ax_3d
        ax.set_title('3D结构与坐标系', fontweight='bold', color='white')

        # 逐段画连杆并分色
        for i in range(len(joint_positions) - 1):
            x = [joint_positions[i][0], joint_positions[i+1][0]]
            y = [joint_positions[i][1], joint_positions[i+1][1]]
            z = [joint_positions[i][2], joint_positions[i+1][2]]
            ax.plot(x, y, z, 'o-', linewidth=3, markersize=6, color=self.link_colors[i % len(self.link_colors)])

        # 基座黑点
        ax.plot([joint_positions[0][0]], [joint_positions[0][1]], [joint_positions[0][2]],
                'ko', markersize=10)

        # 坐标系：使用 quiver（世界 + 关节/末端）
        self._plot_coordinate_frames(Ts)

        # 各连杆质心
        if self.show_com:
            ax.scatter(
                com_positions[:, 0], com_positions[:, 1], com_positions[:, 2],
                c='yellow', s=(0.008 * 1000) ** 2, marker='o',
                depthshade=True, edgecolors='k', linewidths=0.3
            )
        
        # 总质心
        if self.show_total_com:
            total_mass = np.sum(self.robot_model.masses)
            total_com = np.sum(self.robot_model.masses.reshape(-1, 1) * com_positions, axis=0) / total_mass
            ax.scatter(
                total_com[0], total_com[1], total_com[2],
                c='red', s=150, marker='*',
                depthshade=True, edgecolors='white', linewidths=1.5
            )
            ax.text(total_com[0], total_com[1], total_com[2] + 0.05, "总CoM", 
                   fontsize=10, color='red', fontweight='bold')

        # 关节标注 J1..J6、末端 EE
        for i in range(1, len(joint_positions)):
            x, y, z = joint_positions[i]
            ax.text(x, y, z, f"J{i}", fontsize=10, color='white')
        ee_o = Ts[-1][:3, 3]
        ax.text(ee_o[0], ee_o[1], ee_o[2], "EE", fontsize=10, color='white')

        # —— 固定"正方体"坐标系：X/Y ±0.4 m；Z=[0,0.8]（基座在底部中线） —— #
        self._set_axes_fixed_cube(ax, joint_positions)

        ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white'); ax.set_zlabel('Z (m)', color='white')
        ax.grid(True, alpha=0.25, color='white')

        try:
            ax.set_box_aspect((1, 1, 1))  # 正方体显示
        except Exception:
            pass

    def _plot_coordinate_frames(self, Ts):
        """世界坐标与各关节/末端坐标系采用 quiver 箭头 + X/Y/Z 字符；长度固定"""
        ax = self.ax_3d
        L = self.frame_length

        # 世界坐标系（原点）
        if self.show_world_frame:
            ax.quiver(0, 0, 0, L, 0, 0, color='red',   arrow_length_ratio=0.1)
            ax.quiver(0, 0, 0, 0, L, 0, color='green', arrow_length_ratio=0.1)
            ax.quiver(0, 0, 0, 0, 0, L, color='blue',  arrow_length_ratio=0.1)
            ax.text(L*1.05, 0, 0, "X", color='red')
            ax.text(0, L*1.05, 0, "Y", color='green')
            ax.text(0, 0, L*1.05, "Z", color='blue')

        # 关节/末端坐标系
        for i in range(1, len(Ts)):
            is_ee = (i == len(Ts) - 1)
            if (not self.show_joint_frames and not is_ee) or (is_ee and not self.show_ee_frame):
                continue
            T = Ts[i]
            o = T[:3, 3]
            R = T[:3, :3]
            ax.quiver(o[0], o[1], o[2], *(R[:, 0] * L), color='red',   arrow_length_ratio=0.1)
            ax.quiver(o[0], o[1], o[2], *(R[:, 1] * L), color='green', arrow_length_ratio=0.1)
            ax.quiver(o[0], o[1], o[2], *(R[:, 2] * L), color='blue',  arrow_length_ratio=0.1)

    def _set_axes_fixed_cube(self, ax, joints):
        """
        固定坐标系为正方体：
        - X/Y：以基座为中心，[-0.4, +0.4] m
        - Z：以基座为底，从 0 到 +0.8 m
        """
        base = joints[0]
        half = self.fixed_xy_range / 2.0
        ax.set_xlim(base[0] - half, base[0] + half)
        ax.set_ylim(base[1] - half, base[1] + half)
        ax.set_zlim(base[2],        base[2] + self.fixed_z_range)

# ================= 控制面板 =================
class ControlPanel(QWidget):
    pose_changed = pyqtSignal(str)
    joint_changed = pyqtSignal(int, float)
    frames_toggled = pyqtSignal(bool, bool, bool)
    com_toggled = pyqtSignal(bool)
    total_com_toggled = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.robot_model = RobotModel()
        self.sliders = []
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout()

        # 预设
        pose_group = QGroupBox("预设位置")
        pose_layout = QVBoxLayout()
        self.pose_combo = QComboBox()
        self.pose_combo.addItems(list(self.robot_model.test_poses.keys()))
        self.pose_combo.currentTextChanged.connect(self._on_pose_changed)
        pose_layout.addWidget(QLabel("选择预设:"))
        pose_layout.addWidget(self.pose_combo)
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)

        # 显示选项
        frame_group = QGroupBox("显示选项")
        fl = QGridLayout()
        self.cb_world = QCheckBox("世界坐标（X红/Y绿/Z蓝）"); self.cb_world.setChecked(True)
        self.cb_joint = QCheckBox("关节坐标系"); self.cb_joint.setChecked(True)
        self.cb_ee    = QCheckBox("末端坐标系"); self.cb_ee.setChecked(True)
        self.cb_com   = QCheckBox("连杆质心");   self.cb_com.setChecked(True)
        self.cb_total_com = QCheckBox("总质心"); self.cb_total_com.setChecked(True)
        
        self.cb_world.stateChanged.connect(self._on_toggle_frames)
        self.cb_joint.stateChanged.connect(self._on_toggle_frames)
        self.cb_ee.stateChanged.connect(self._on_toggle_frames)
        self.cb_com.stateChanged.connect(lambda _=None: self.com_toggled.emit(self.cb_com.isChecked()))
        self.cb_total_com.stateChanged.connect(lambda _=None: self.total_com_toggled.emit(self.cb_total_com.isChecked()))
        
        fl.addWidget(self.cb_world, 0, 0)
        fl.addWidget(self.cb_joint, 0, 1)
        fl.addWidget(self.cb_ee,    1, 0)
        fl.addWidget(self.cb_com,   1, 1)
        fl.addWidget(self.cb_total_com, 2, 0)
        frame_group.setLayout(fl)
        layout.addWidget(frame_group)

        # 关节滑条
        joint_group = QGroupBox("关节角度控制（°）")
        gl = QGridLayout()
        for i in range(6):
            gl.addWidget(QLabel(f"关节{i+1}:"), i, 0)
            s = QSlider(Qt.Horizontal); s.setRange(-180, 180); s.setValue(0)
            s.valueChanged.connect(lambda v, idx=i: self._on_joint_changed(idx, v))
            self.sliders.append(s); gl.addWidget(s, i, 1)
            val = QLabel("0°"); val.setMinimumWidth(48); val.setAlignment(Qt.AlignCenter)
            s.valueChanged.connect(lambda v, lab=val: lab.setText(f"{v}°"))
            gl.addWidget(val, i, 2)
        joint_group.setLayout(gl)
        layout.addWidget(joint_group)

        # 重置
        btns = QHBoxLayout()
        reset_btn = QPushButton("重置到零位"); reset_btn.clicked.connect(self._reset_zero)
        btns.addWidget(reset_btn)
        layout.addLayout(btns)

        self.setLayout(layout)

    def _on_pose_changed(self, name):
        if name in self.robot_model.test_poses:
            ang = self.robot_model.test_poses[name]
            for i, a in enumerate(ang):
                self.sliders[i].setValue(int(np.degrees(a)))
            self.pose_changed.emit(name)

    def _on_toggle_frames(self):
        self.frames_toggled.emit(self.cb_joint.isChecked(), self.cb_world.isChecked(), self.cb_ee.isChecked())

    def _on_joint_changed(self, idx, deg):
        self.joint_changed.emit(idx, np.radians(deg))

    def _reset_zero(self):
        for s in self.sliders:
            s.setValue(0)

# ================= 信息面板 =================
class InfoPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.robot_model = RobotModel()
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout()
        tabs = QTabWidget()

        # 运动学
        kin_tab = QWidget(); kin_layout = QVBoxLayout()
        self.joint_table = QTableWidget(7, 4)
        self.joint_table.setHorizontalHeaderLabels(["位置", "X (m)", "Y (m)", "Z (m)"])
        self.joint_table.setVerticalHeaderLabels(["基座","关节1","关节2","关节3","关节4","关节5","关节6"])
        kin_layout.addWidget(QLabel("关节位置:"))
        kin_layout.addWidget(self.joint_table)
        kin_tab.setLayout(kin_layout)
        tabs.addTab(kin_tab, "运动学")

        # 重力补偿（重新激活并显示真实计算）
        grav_tab = QWidget(); grav_layout = QVBoxLayout()
        
        # 重力补偿力矩表
        self.gravity_table = QTableWidget(6, 3)
        self.gravity_table.setHorizontalHeaderLabels(["关节", "重力力矩 (N·m)", "质量 (kg)"])
        for i in range(6):
            self.gravity_table.setItem(i, 0, QTableWidgetItem(f"关节{i+1}"))
            self.gravity_table.setItem(i, 1, QTableWidgetItem("0.0000"))
            self.gravity_table.setItem(i, 2, QTableWidgetItem(f"{self.robot_model.masses[i]:.1f}"))
        grav_layout.addWidget(QLabel("重力补偿力矩:"))
        grav_layout.addWidget(self.gravity_table)

        # 动力学信息文本
        self.info_text = QTextEdit(); self.info_text.setMaximumHeight(260)
        grav_layout.addWidget(QLabel("动力学信息（世界坐标：X红/Y绿/Z蓝）:"))
        grav_layout.addWidget(self.info_text)
        grav_tab.setLayout(grav_layout)
        tabs.addTab(grav_tab, "重力补偿")

        layout.addWidget(tabs)
        self.setLayout(layout)

    def update_info(self, theta):
        joint_positions, Ts = self.robot_model.get_joint_positions(theta)
        dynamic_info = self.robot_model.get_dynamic_info(theta)

        # 关节位置表
        for i, p in enumerate(joint_positions):
            self.joint_table.setItem(i, 0, QTableWidgetItem("基座" if i==0 else f"关节{i}"))
            self.joint_table.setItem(i, 1, QTableWidgetItem(f"{p[0]:.4f}"))
            self.joint_table.setItem(i, 2, QTableWidgetItem(f"{p[1]:.4f}"))
            self.joint_table.setItem(i, 3, QTableWidgetItem(f"{p[2]:.4f}"))

        # 重力补偿力矩表
        gravity_torques = dynamic_info['gravity_torques']
        for i in range(6):
            self.gravity_table.setItem(i, 0, QTableWidgetItem(f"关节{i+1}"))
            self.gravity_table.setItem(i, 1, QTableWidgetItem(f"{gravity_torques[i]:.4f}"))
            self.gravity_table.setItem(i, 2, QTableWidgetItem(f"{self.robot_model.masses[i]:.1f}"))

        # 文本信息
        T6 = Ts[-1]
        pos = T6[:3,3]
        rpy = self.robot_model.rot_to_rpy_zyx(T6[:3,:3])
        info = []
        info.append(f"末端位置 (m): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
        info.append(f"末端欧拉角 ZYX (rad): [yaw {rpy[0]:.4f}, pitch {rpy[1]:.4f}, roll {rpy[2]:.4f}]")
        info.append("末端齐次矩阵 T06:")
        mat_lines = [f"[{T6[i,0]: .4f} {T6[i,1]: .4f} {T6[i,2]: .4f} {T6[i,3]: .4f}]" for i in range(4)]
        info.extend(mat_lines)
        info.append("")
        info.append("=== 动力学参数 ===")
        info.append(f"总质量: {dynamic_info['total_mass']:.2f} kg")
        info.append(f"总质心位置 (m): [{dynamic_info['total_com'][0]:.4f}, {dynamic_info['total_com'][1]:.4f}, {dynamic_info['total_com'][2]:.4f}]")
        info.append(f"重力势能: {dynamic_info['potential_energy']:.2f} J")
        info.append("")
        info.append("=== 重力补偿力矩 ===")
        for i, torque in enumerate(gravity_torques):
            info.append(f"关节{i+1}: {torque:8.4f} N·m (质量: {self.robot_model.masses[i]:.1f} kg)")
        info.append("")
        info.append("世界坐标轴：X=红、Y=绿、Z=蓝（右手系）")
        info.append(f"当前关节角度 (度): [{', '.join([f'{np.degrees(a):.1f}' for a in theta])}]")
        info.append("")
        info.append("重力补偿说明：")
        info.append("- 基于τ_g = J_v^T * F_g方法计算")
        info.append("- 考虑各连杆质量和质心位置")
        info.append("- 正值表示需要正向驱动力矩")
        info.append("- 负值表示需要反向驱动力矩")
        
        # 质心本地不变量自检
        max_err, per_link_errs = self.robot_model.check_com_invariance(theta)
        info.append(f"CoM相对不变性最大误差: {max_err*1000:.3f} mm")
        self.info_text.setPlainText("\n".join(info))

# ================= 主窗口 =================
class MainWindow(QMainWindow):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.robot_model = RobotModel()
        self._build_ui()

    def _build_ui(self):
        self.setWindowTitle("机械臂可视化（重力补偿计算 | 固定0.8 m正方体坐标系）")
        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            QLabel { color: #E0E0E0; }
            QGroupBox {
                color: #E0E0E0; font-weight: bold;
                border: 1px solid #2b2b2b; border-radius: 10px;
                margin-top: 8px; padding: 10px;
                background-color: #1a1a1a;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QComboBox, QTableWidget, QTextEdit {
                background-color: #181818; color: #E0E0E0; border: 1px solid #2b2b2b;
            }
            QSlider::groove:horizontal { height: 6px; background: #2b2b2b; border-radius: 3px; }
            QSlider::handle:horizontal { background: #00BFA5; width: 14px; margin: -5px 0; border-radius: 7px; }
            QSlider::sub-page:horizontal { background: #00BFA5; }
            QPushButton {
                background-color: #00BFA5; color: #111; border-radius: 8px; padding: 8px 14px; font-weight: bold;
            }
            QPushButton:hover { background-color: #00D1B2; }
            QTabWidget::pane { border: 1px solid #2b2b2b; }
            QTabBar::tab { background: #1a1a1a; color: #E0E0E0; padding: 6px 12px; border: 1px solid #2b2b2b; }
            QTabBar::tab:selected { background: #222; }
            QCheckBox { color: #E0E0E0; }
        """)

        # 屏幕尺寸与比例自适应
        screen = self.app.primaryScreen()
        avail = screen.availableGeometry()
        w_total = avail.width()
        # 左中右比例：控制面板 : 3D画布 : 信息面板
        left_w  = max(300, int(w_total * 0.20))
        right_w = max(320, int(w_total * 0.24))
        center_w= max(800, int(w_total * 0.56))

        central = QWidget(); self.setCentralWidget(central)
        main_layout = QHBoxLayout()

        splitter = QSplitter(Qt.Horizontal)
        self.control_panel = ControlPanel(); self.control_panel.setMaximumWidth(left_w+40)
        self.canvas = RobotCanvas()
        self.info_panel = InfoPanel(); self.info_panel.setMaximumWidth(right_w+40)

        splitter.addWidget(self.control_panel)
        splitter.addWidget(self.canvas)
        splitter.addWidget(self.info_panel)
        splitter.setSizes([left_w, center_w, right_w])

        main_layout.addWidget(splitter)
        central.setLayout(main_layout)

        # 信号连接
        self.control_panel.pose_changed.connect(self._on_pose_changed)
        self.control_panel.joint_changed.connect(self._on_joint_changed)
        self.control_panel.frames_toggled.connect(self._on_frames_toggled)
        self.control_panel.com_toggled.connect(self._on_com_toggled)
        self.control_panel.total_com_toggled.connect(self._on_total_com_toggled)

        # 初始窗口尺寸并最大化
        self.setGeometry(avail)
        self.showMaximized()

        self._refresh()

    def _on_pose_changed(self, name):
        if name in self.robot_model.test_poses:
            self.robot_model.current_theta = np.array(self.robot_model.test_poses[name], dtype=float)
            self._refresh()

    def _on_joint_changed(self, idx, rad):
        th = self.robot_model.current_theta.copy()
        th[idx] = rad
        self.robot_model.current_theta = th
        self._refresh()

    def _on_frames_toggled(self, show_joint, show_world, show_ee):
        self.canvas.set_frame_toggles(show_joint, show_world, show_ee)

    def _on_com_toggled(self, show):
        self.canvas.set_com_toggle(show)
    
    def _on_total_com_toggled(self, show):
        self.canvas.set_total_com_toggle(show)

    def _refresh(self):
        self.canvas.robot_model.current_theta = self.robot_model.current_theta.copy()
        self.canvas.update_plot()
        self.info_panel.update_info(self.robot_model.current_theta)

# ================= 入口 =================
def main():
    # 高DPI与自适应（需在 QApplication 前设置）
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setApplicationName("机械臂重力补偿可视化系统")
    app.setApplicationVersion("3.0")
    app.setFont(QFont("Microsoft YaHei", 10))

    matplotlib.rcParams['figure.facecolor'] = '#0f0f0f'
    matplotlib.rcParams['axes.facecolor']   = '#1e1e1e'
    matplotlib.rcParams['savefig.facecolor']= '#0f0f0f'

    w = MainWindow(app)
    w.show()  # showMaximized 已在 MainWindow 中调用
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()