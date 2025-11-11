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

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
from time import sleep

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import spatialgeometry as sg
from roboticstoolbox import Robot

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        if startup_check:
            self.startup_check()
        else:
            home =  np.radians(np.array([[50,-125,55,-130,-20,-65, 85], [-50,-125,-55,-130,20,-65, 85]], dtype=np.float32))

            angles = [ifaces.RoboCompKinovaArm.TJointAngles(jointAngles=ifaces.RoboCompKinovaArm.Angles(np.array(home[0]))), 
                        ifaces.RoboCompKinovaArm.TJointAngles(jointAngles=ifaces.RoboCompKinovaArm.Angles(np.array(home[1])))]
            self.kinovaarm_proxy.moveJointsWithAngle(angles[0])
            self.kinovaarm1_proxy.moveJointsWithAngle(angles[1])

            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            self.p3bot = rtb.models.P3Bot()

            T = sm.SE3(0, 0, 0.04)
            Rz = sm.SE3.Rz(1.57)
            self.p3bot.base = T * Rz
            self.env.add(self.p3bot)

            sleep(10)

            data_right = self.kinovaarm_proxy.getJointsState()
            angles_right = np.array([joint.angle for joint in data_right.joints])

            data_left = self.kinovaarm1_proxy.getJointsState()
            angles_left = np.array([joint.angle for joint in data_left.joints])

            self.p3bot.q[2 + 0 * 7 : 9 + 0 * 7] = angles_right
            self.p3bot.q[2 + 1 * 7 : 9 + 1 * 7] = angles_left
            self.env.step()

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):

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
        QTimer.singleShot(200, QApplication.instance().quit)





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
    # RoboCompKinovaArm.bool self.kinovaarm_proxy.setGripperPos(float pos)

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
    # RoboCompKinovaArm.bool self.kinovaarm1_proxy.setGripperPos(float pos)

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


