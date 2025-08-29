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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./generated/ --all ./generated/VRControllerPub.ice")

from RoboCompVRControllerPub import *

class VRControllerPubI(VRControllerPub):
    def __init__(self, worker, id:str):
        self.worker = worker
        self.id = id


    def sendControllers(self, left, right, ice):
        return getattr(self.worker, f"VRControllerPub{self.id}_sendControllers")(left, right)

    def sendPose(self, head, left, right, ice):
        return getattr(self.worker, f"VRControllerPub{self.id}_sendPose")(head, left, right)
