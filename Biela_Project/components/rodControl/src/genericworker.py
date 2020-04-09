#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
	ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

additionalPathStr = ''
icePaths = [ '/opt/robocomp/interfaces' ]
try:
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
	icePaths.append('/opt/robocomp/interfaces')
except:
	print('SLICE_PATH environment variable was not exported. Using only the default paths')
	pass

ice_CameraRGBDSimple = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimple.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimple.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimple = True
		break
if not ice_CameraRGBDSimple:
	print('Couln\'t load CameraRGBDSimple')
	sys.exit(-1)
from RoboCompCameraRGBDSimple import *




class GenericWorker(QtCore.QObject):

	kill = QtCore.Signal()
#Signals for State Machine
	t_initialize_to_detectCircle = QtCore.Signal()
	t_detectCircle_to_moveArm = QtCore.Signal()
	t_moveArm_to_takeRod = QtCore.Signal()
	t_takeRod_to_moveRod = QtCore.Signal()
	t_moveRod_to_dropRod = QtCore.Signal()
	t_dropRod_to_finalize = QtCore.Signal()

#-------------------------

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


		self.camerargbdsimple_proxy = mprx["CameraRGBDSimpleProxy"]

		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

#State Machine
		self.rodMachine= QtCore.QStateMachine()
		self.detectCircle_state = QtCore.QState(self.rodMachine)
		self.moveArm_state = QtCore.QState(self.rodMachine)
		self.takeRod_state = QtCore.QState(self.rodMachine)
		self.moveRod_state = QtCore.QState(self.rodMachine)
		self.dropRod_state = QtCore.QState(self.rodMachine)
		self.initialize_state = QtCore.QState(self.rodMachine)

		self.finalize_state = QtCore.QFinalState(self.rodMachine)


#------------------
#Initialization State machine
		self.initialize_state.addTransition(self.t_initialize_to_detectCircle, self.detectCircle_state)
		self.detectCircle_state.addTransition(self.t_detectCircle_to_moveArm, self.moveArm_state)
		self.moveArm_state.addTransition(self.t_moveArm_to_takeRod, self.takeRod_state)
		self.takeRod_state.addTransition(self.t_takeRod_to_moveRod, self.moveRod_state)
		self.moveRod_state.addTransition(self.t_moveRod_to_dropRod, self.dropRod_state)
		self.dropRod_state.addTransition(self.t_dropRod_to_finalize, self.finalize_state)


		self.detectCircle_state.entered.connect(self.sm_detectCircle)
		self.moveArm_state.entered.connect(self.sm_moveArm)
		self.takeRod_state.entered.connect(self.sm_takeRod)
		self.moveRod_state.entered.connect(self.sm_moveRod)
		self.dropRod_state.entered.connect(self.sm_dropRod)
		self.initialize_state.entered.connect(self.sm_initialize)
		self.finalize_state.entered.connect(self.sm_finalize)

		self.rodMachine.setInitialState(self.initialize_state)

#------------------

#Slots funtion State Machine
	@QtCore.Slot()
	def sm_detectCircle(self):
		print("Error: lack sm_detectCircle in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_moveArm(self):
		print("Error: lack sm_moveArm in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_takeRod(self):
		print("Error: lack sm_takeRod in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_moveRod(self):
		print("Error: lack sm_moveRod in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_dropRod(self):
		print("Error: lack sm_dropRod in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_initialize(self):
		print("Error: lack sm_initialize in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_finalize(self):
		print("Error: lack sm_finalize in Specificworker")
		sys.exit(-1)


#-------------------------
	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print("Period changed", p)
		self.Period = p
		self.timer.start(self.Period)
