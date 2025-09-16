#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import threading
from time import sleep
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from rich.text import Text
from rich.table import Table
from rich.live import Live
from collections import deque

from genericworker import *
import interfaces as ifaces
import math

from scipy.spatial.transform import Rotation as R



import swift
import roboticstoolbox as rtb
import spatialmath as sm
import qpsolvers as qp
import spatialgeometry as sg
import numpy as np


SCALE = 0.001
console = Console(highlight=False)
live = Live(console=console, refresh_per_second=10)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.simulated = self.configData["simulated"]
        self.directKinematic = self.configData["directKinematic"]

        assert isinstance(self.simulated, bool), f"Simulated must be bool, dont {type(self.simulated)}"
        assert isinstance(self.directKinematic, bool), f"directKinematic must be bool, dont {type(self.directKinematic)}"



        self.pose = None
        # self.cubes_positions = [sm.SE3.Trans(0.0, 0.0, 0.20), sm.SE3.Trans(-0.10, 0, 0.7), sm.SE3.Trans(-0.125, -0., 1)]
        # self.collisions = [sg.Cuboid((0.46, 0.46, 0.40), pose=self.cubes_positions[0], color=(1, 0, 0)),
        #                 sg.Cuboid((0.20, 0.20, 0.750), pose=self.cubes_positions[1], color=(1, 0, 0)),
        #                 sg.Cuboid((0.10, 0.10, 0.750), pose=self.cubes_positions[2], color=(1, 0, 0))]
        
        self.gain = np.array([1, 1, 1, 1.6, 1.6, 1.6])

        self.kinova_pub_arms = [self.kinovaarmpub_proxy, self.kinovaarmpub1_proxy]
        self.kinova_arms = [None, None]
        if not self.simulated:
            self.kinova_arms = [self.kinovaarm_proxy, self.kinovaarm1_proxy]

        if startup_check:
            self.startup_check()
        else:
            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            # for colision in self.collisions:
            #     self.env.add(colision)

            #region P3Bot
            # self.p3bot = Robot.URDF("/home/robolab/software/robotics-toolbox-python/rtb-data/rtbdata/xacro/p3bot_description/urdf/P3Bot_scaled.urdf")
            self.p3bot = rtb.models.P3Bot()
            print(self.p3bot.qdlim)

            self.p3bot.qdlim = [0]*2 + [1.5]*14
            print(self.p3bot.qdlim)

            self.bodyOffset = sm.SE3.Rz(3.14)
            self.targetOffset = sm.SE3.Rz(np.deg2rad(180)) * sm.SE3.Ry(np.deg2rad(180)) 

            T = sm.SE3(0, 0, 0.04)
            Rz = sm.SE3.Rz(3.14)
            self.p3bot.base = T * Rz
            self.env.add(self.p3bot)
            #endregion
            #region Tool Points
            self.collisions_tool = []
            for i in range(2):
                self.p3bot.grippers[i].tool *= sm.SE3.Tz(0.135) * sm.SE3.Rz(np.deg2rad(180))
                frame = sg.Axes(0.1, pose=self.p3bot.grippers[i].tool)
                frame.attach_to(self.p3bot.grippers[i].links[0])
                self.env.add(frame)

                self.collisions_tool.append(sg.Cuboid((0.12, 0.15, 0.2), pose=self.p3bot.grippers[i].tool, color=(1, 0, 0,0.25)),)
                self.env.add(self.collisions_tool[i])
            #endregion

            #region getGoal
            self.goal_axes = [sg.Axes(0.1), sg.Axes(0.1)]
            self.deadManButton = [False, False]
            self.target = [None, None]
            self.poseController = [np.array((0,0,0,0,0,0)), np.array((0,0,0,0,0,0))]
            #endregion

            self.home =  np.radians(np.array([[40,-120,60,-130,-20,-65, 85], [-40,-120,-60,-130,20,-65, 85]], dtype=np.float64))
            self.pick =  np.radians(np.array([[90,-120,80,-130,-20, 45, 95], [-90,-120,-80,-130, 20, 45, 85]], dtype=np.float64))
            self.set_all_joints(self.pick)
            for i in range(2): self.update_collisions_tool(self.p3bot, i)


            
            # live.start()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            

    def __del__(self):
        """Destructor"""

    def generate_test_status_table(self) -> Table:
        table = Table(title="Estado")
        table.add_column("Axis")
        table.add_column("Robot pose")
        table.add_column("Robot target", style="green")
        table.add_column("Right pose")
        table.add_column("Right target", style="green" if self.deadManButton[0] else None)
        table.add_column("Left pose")
        table.add_column("Left target", style="green" if self.deadManButton[1] else None)

        base_t = self.p3bot.base.t
        base_rpy = self.p3bot.base.rpy()

        tool0 = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[0])
        tool1 = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[1])
        tool0_t, tool0_rpy = tool0.t, tool0.rpy()
        tool1_t, tool1_rpy = tool1.t, tool1.rpy()

        target0 = self.target[0]
        target1 = self.target[1]
        target0_t = target0.t if target0 is not None else ["", "", ""]
        target1_t = target1.t if target1 is not None else ["", "", ""]
        target0_rpy = target0.rpy() if target0 is not None else ["", "", ""]
        target1_rpy = target1.rpy() if target1 is not None else ["", "", ""]

        # posiciones
        for i, axis in enumerate(("x", "y", "z")):
            table.add_row(
                axis,
                f"{base_t[i]:.3f}",
                f"{base_t[i]:.3f}",
                f"{tool0_t[i]:.3f}",
                f"{target0_t[i]:.3f}" if target0 else "",
                f"{tool1_t[i]:.3f}",
                f"{target1_t[i]:.3f}" if target1 else ""
            )

        # orientaciones
        for i, axis in enumerate(("rx", "ry", "rz")):
            table.add_row(
                axis,
                f"{base_rpy[i]:.3f}",
                f"{base_rpy[i]:.3f}",
                f"{tool0_rpy[i]:.3f}",
                f"{target0_rpy[i]:.3f}" if target0 else "",
                f"{tool1_rpy[i]:.3f}",
                f"{target1_rpy[i]:.3f}" if target1 else ""
            )

        return table

    @QtCore.Slot()
    def compute(self):
        # #Update pose in swift
        if self.pose is not None:
            T = sm.SE3(self.pose[0:3])
            # T = sm.SE3()
            q = sm.UnitQuaternion(self.pose[3:])
            R = q.SE3()
            self.pose = None
            self.p3bot.base = T *self.bodyOffset * R
        for arm in range(len(self.kinova_pub_arms)):
            self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
            self.kinova_pub_arms[arm].sendJointsWithAngle(ifaces.RoboCompKinovaArm.TJointAngles(np.array(self.p3bot.q[2 + arm * 7 : 9 + arm * 7], dtype=np.float32)))

        # self.update_collisions(self.p3bot.base)
    
        #Go to target
        for arm in range(2):
            if self.deadManButton[arm]:
                self.change_target(arm, self.poseController[arm][:3], self.poseController[arm][3:])
                self.update_collisions_tool(self.p3bot, arm)

                if self.directKinematic:                
                    arrived, qd = self.direct_kinematic_robot(self.p3bot, arm, self.target[arm].A)
                else:
                    arrived, qd = self.step_robot(self.p3bot, arm, self.target[arm].A, self.collisions_tool[arm])


                # qd[2:] = [0]*(len(qd)-2)

                #Move motors
                if qd is not None:
                    self.set_velocity_joints(arm, qd)
                    
                if arrived:
                    print(f"arm {arm} ARRIBEEEEEDDDD")
                    self.set_velocity_joints(arm, [0]*7)
            else:
                self.set_velocity_joints(arm, [0]*7)


        self.env.step(0.05)

        base_new = self.p3bot.fkine(self.p3bot._q, end=self.p3bot.links[2])
        self.p3bot._T = base_new.A
        self.p3bot.q[:2] = 0
        live.update(self.generate_test_status_table())
        return True

    def startup_check(self):
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TAxis from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TAxis()
        print(f"Testing RoboCompKinovaArm.TToolInfo from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TToolInfo()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompKinovaArm.TJointSpeeds from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointSpeeds()
        print(f"Testing RoboCompKinovaArm.TJointAngles from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointAngles()
        print(f"Testing RoboCompVRControllerPub.Pose from ifaces.RoboCompVRControllerPub")
        test = ifaces.RoboCompVRControllerPub.Pose()
        print(f"Testing RoboCompVRControllerPub.Controller from ifaces.RoboCompVRControllerPub")
        test = ifaces.RoboCompVRControllerPub.Controller()
        QTimer.singleShot(200, QApplication.instance().quit)

    def set_all_joints(self, poses: list[list[float]]) -> None:
        """Mueve todos los brazos robóticos a los ángulos articulares especificados en paralelo.
        
        Args:
            poses (list[list[float]]): Ángulos objetivo para cada brazo (en radianes).
        
        Raises:
            AssertionError: Si el número de poses no coincide con el número de brazos.
        """
        assert len(poses) == len(self.kinova_arms), \
            f"El robot tiene {len(self.kinova_arms)} brazos, pero se proporcionaron {len(poses)} poses"

        threads = []
        for i in range(len(self.kinova_arms)):
            # Crear un hilo por brazo
            thread = threading.Thread(
                target=self.set_joints,
                args=(i, poses[i])  # Argumentos: (arm, pose)
            )
            thread.start()
            threads.append(thread)  # Guardar referencia

        # Esperar a que todos los hilos terminen
        for thread in threads:
            thread.join()
                
    def set_joints(self, arm: int, pose: list[float]) -> None:
        """Move the robot arm to the specified joint angles.

        Args:
            arm (int): Index of the robot arm (0-based).
            pose (list[float]): Target joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If setting joint angles fails (logged to console).
        """
        assert arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        assert 7 == len(pose), f"Robot has {7} joins, tried use {len(pose)}" 
        try:
            counter = 0
            while not np.allclose(self.p3bot.q[2 + arm * 7 : 9 + arm * 7], pose, rtol=0.0001):
                if counter % 1000 == 0:
                    console.print(Text(f"Set pose {pose}", "green"))
                    if self.simulated:
                        self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = pose
                    else:
                        self.set_velocity_joints(arm, [0]*7)
                        angles = ifaces.RoboCompKinovaArm.TJointAngles(jointAngles=ifaces.RoboCompKinovaArm.Angles(np.array(pose)))
                        self.kinova_arms[arm].moveJointsWithAngle(angles)

                
                sleep(0.005)
                self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
                # self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = pose
                # print(self.p3bot.q[2 + arm * 7 : 9 + arm * 7], pose, "\n\n\n")
                self.env.step(0)
                counter+=1
        except Exception as e:
            console.print(Text(f"Failed to set joint angles: {e}", "red"))
            console.print_exception()
        finally:
            self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
            self.env.step(0)




    def set_velocity_joints(self, arm: int, velocity: list[float]) -> None:
            """Set the velocity of the robot arm joints.

            Args:
                arm (int): Index of the robot arm (0-based).
                velocity (list[float]): List of velocities (in rad/s) for each joint.

            Raises:
                AssertionError: If the arm index is out of bounds.
                Exception: If setting velocity fails (logged to console).
            """
            assert self.simulated or arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
            assert 7 == len(velocity), f"Robot has {7} joins, tried use {len(velocity)}" 
            try:
                # console.print(Text(f"Set velocity {velocity}", "green"))

                if not self.simulated:
                    speed = ifaces.RoboCompKinovaArm.TJointSpeeds(jointSpeeds=ifaces.RoboCompKinovaArm.Speeds(velocity))
                    self.kinova_arms[arm].moveJointsWithSpeed(speed)
                self.p3bot.qd[2 + arm * 7 : 9 + arm * 7] = velocity
            
            except Exception as e:
                console.print(Text(f"Failed to set joint velocities: {e}", "red"))
                console.print_exception()
        # finally:
        #     self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)

    def get_joints(self, arm: int) -> list[float]:
        """Retrieve the current joint angles of the robot arm.

        Args:
            arm (int): Index of the robot arm (0-based).

        Returns:
            list[float]: Current joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If fetching joint angles fails (logged to console).            q = sm.UnitQuaternion(rot)
            R = q.SE3()
        """
        assert self.simulated or arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        try:
            if self.simulated:
                    return self.p3bot.q[2 + arm * 7 : 9 + arm * 7].tolist()

            else:
                data = self.kinova_arms[arm].getJointsState()
                angles = np.array([joint.angle for joint in data.joints])
                angles[angles > np.pi] -= 2*np.pi  # Normalize angles >180° to [-180°, 180°]
                return angles.tolist()
        except Exception as e:
            console.print(Text(f"Failed to get joint angles: {e}", "red"))
            console.print_exception()
            return []
        
    def update_collisions(self, pose:sm.SE3.Trans):
        for i in range(len(self.collisions)):
            self.collisions[i].T = pose * self.cubes_positions[i]

    def update_collisions_tool(self, r:rtb.ERobot, gripperSelect:int):
        gripper = r.grippers[gripperSelect]   # por ejemplo
        wTe = r.fkine(r.q, end=gripper)
        self.collisions_tool[gripperSelect].T = wTe * sm.SE3([-0.02, 0, -0.07])
        

    def change_target(self, arm:int, translate:np.ndarray, rot:np.ndarray):
        if self.deadManButton[arm]:
            T = sm.SE3(translate*SCALE)
            q = sm.UnitQuaternion(rot)
            R = q.SE3()
            
            self.target[arm] = T * self.targetOffset * R 
            self.goal_axes[arm].T = self.target[arm]
            self.env.add(self.goal_axes[arm])


    def direct_kinematic_robot(self, r: rtb.ERobot, gripperSelect, Tep):
        gripper = r.grippers[gripperSelect]

        ets = r.ets(end=gripper)
        indices = ets.jindices

        v, arrived = rtb.p_servo(r.fkine(r.q, end=gripper), Tep, gain=self.gain, threshold=0.005)
        qd = np.clip(np.linalg.pinv(ets.jacobe(r.q)) @ v, -r.qdlim[indices], r.qdlim[indices])[2:9]
                                        
        return arrived, qd
        
    def step_robot(self, r: rtb.ERobot, gripperSelect, Tep, collision):
        collision_body = [["low_body", "right_arm_spherical_wrist_2_link"],["left_arm", "left_arm_spherical_wrist_2_link"]]
        # colli = [["right_arm_shoulder_link", "right_arm_spherical_wrist_2_link"],["left_arm_shoulder_link", "left_arm_spherical_wrist_2_link"]]
        n = 9
        gripper = r.grippers[gripperSelect]
        ets = r.ets(end=gripper)
        wTe = r.fkine(r.q, end=gripper)

        eTep = np.linalg.inv(wTe) @ Tep

        # Spatial error
        et = np.sum(np.abs(eTep[:3, -1]))

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(n + 6)

        # Joint velocity component of Q
        Q[: n, : n] *= Y
        Q[:3, :3] *= 1.0 / et

        # Slack component of Q
        Q[n:, n:] = (1.0 / et) * np.eye(6)

        v, _ = rtb.p_servo(wTe, Tep, 1.5)

        v[3:] *= 1.3

        # The equality contraints
        Aeq = np.c_[ets.jacobe(r.q), np.eye(6)]#TODO tool
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.1

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[: n, : n], bin[: n] = r.joint_velocity_damper(ps, pi, n)

        rot_boost = 1
        vel_decay = 1

        #################COLISIONS##################
        for i, body in enumerate(collision_body):
            c_Ain, c_bin = self.p3bot.link_collision_damper(
                    collision,
                    self.p3bot.q,
                    di=0.1, # Distancia mínima más pequeña (ej: 0.1 metros)
                    ds=0.05, # Ganancia más alta (ej: 0.1)
                    xi=1, # Mayor peso en la optimización
                    start= self.p3bot.link_dict[body[0]], 
                    end= self.p3bot.link_dict[body[1]]
                )

            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], n + 6 - c_Ain.shape[1]))]
                print(f"{i}, colision {c_Ain.shape}, {c_bin.shape}")
                # if len(c_Ain) > 1 : vel_decay +=len(c_bin)*2

                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        ############################

        # Linear component of objective function: the manipulability Jacobian
        c = np.concatenate(
            (np.zeros(2), -r.jacobm(start=r.links[3], end=gripper).reshape((n - 2,)), np.zeros(6))
        )

        # # Get base to face end-effector
        # kε = 0.5
        # bTe = r.fkine(r.q, end=gripper, include_base=False).A
        # θε = math.atan2(bTe[1, -1], bTe[0, -1])
        # ε = kε * θε
        # c[0] = -ε

        # The lower and upper bounds on the joint velocity and slack variable
        start = gripperSelect*7
        lb = -np.r_[r.qdlim[:2],r.qdlim[start+2:start+n], 10 * np.ones(6)]
        ub = np.r_[r.qdlim[:2], r.qdlim[start+2:start+n], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="piqp")
        arrived = False

        if qd is not None:
            qd = qd.copy()[2:9]

            if et < 0.02:
                arrived = True
        else:
            console.print(Text("Optimización fallida.", "yellow"))
            qd = np.zeros(7)
        return arrived, qd

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to sendControllers method from VRControllerPub interface
    #
    def VRControllerPub_sendControllers(self, left, right):
        self.deadManButton[1] = left.grab>0.85008
        self.deadManButton[0] = right.grab>0.8
        pass


    #
    # SUBSCRIPTION to sendPoses method from VRControllerPub interface
    #

    def normalize_angle(self, a):
        return (a + math.pi) % (2*math.pi) - math.pi

    def fix_controller_pose(self, x, y, z, rx, ry, rz):
        # Construye la pose del controlador (ajusta la convención si es necesario)
        T_ctrl = sm.SE3(x, y, z) * sm.SE3.RPY(rx, ry, rz)   # o SE3.RPY(..., order='xyz') según convención

        # Rotación fija: Rz(pi) * Ry(-pi/2)
        T_fix = sm.SE3.Rz(math.pi) * sm.SE3.Ry(-math.pi/2)

        # Aplica la corrección (primero la fix, luego la pose del controlador en el mismo marco)
        T_new = T_fix * T_ctrl

        # Extraer traslación y ángulos RPY (en radianes)
        t = T_new.t  # array(3,)
        rpy = T_new.rpy()  # por defecto roll-pitch-yaw

        # Normalizar ángulos
        rpy = np.array([self.normalize_angle(a) for a in rpy])

        # Devuelve vector igual al que tenías: (x, y, z, rx, ry, rz)
        return np.array((t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]))
    

    def fix_controller_orientation(self, x,y,z,rx,ry,rz):
        # Matriz de rotación original
        R_ctrl = R.from_euler('xyz', [rx, ry, rz]).as_matrix()

        # Rotación fija
        R_fix = (R.from_euler('z', math.pi) * R.from_euler('y', -math.pi/2)).as_matrix()

        # Solo reorientar
        R_new = R_fix @ R_ctrl

        # Convertir a Euler (misma convención)
        rx2, ry2, rz2 = R.from_matrix(R_new).as_euler('xyz')

        return np.array((x, y, z, rx2, ry2, rz2))
    
    def VRControllerPub_sendPoses(self, head, left, right):
        # Cambiar a coordenadas ROS
        self.pose = np.array([-head.x*SCALE, head.y*SCALE, head.z*SCALE-1.4,head.qrw, 0, 0, -head.qrz])

        self.poseController[1] = np.array((-left.x, left.y, left.z, left.qrw, left.qrx, left.qry, left.qrz))
        self.poseController[0] = np.array((-right.x, right.y, right.z, right.qrw, right.qrx, right.qry, right.qrz))



    # ===================================================================
    # ===================================================================



    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm_proxy.getJointsState()
    # RoboCompKinovaArm.TToolInfo self.kinovaarm_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm1_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm1_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm1_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm1_proxy.getJointsState()
    # RoboCompKinovaArm.TToolInfo self.kinovaarm1_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompKinovaArmPub you can publish calling this methods:
    # RoboCompKinovaArmPub.void self.kinovaarmpub_proxy.newArmState(RoboCompKinovaArm.TPose armState)
    # RoboCompKinovaArmPub.void self.kinovaarmpub_proxy.sendJointsWithAngle(RoboCompKinovaArm.TJointAngles angles)

    ######################
    # From the RoboCompKinovaArmPub you can publish calling this methods:
    # RoboCompKinovaArmPub.void self.kinovaarmpub1_proxy.newArmState(RoboCompKinovaArm.TPose armState)
    # RoboCompKinovaArmPub.void self.kinovaarmpub1_proxy.sendJointsWithAngle(RoboCompKinovaArm.TJointAngles angles)

    ######################
    # From the RoboCompVRControllerPub you can use this types:
    # ifaces.RoboCompVRControllerPub.Pose
    # ifaces.RoboCompVRControllerPub.Controller


