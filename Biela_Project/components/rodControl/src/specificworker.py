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
#

from genericworker import *
import cv2
import numpy as np
import math
import vrep
import b0RemoteApi
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.Period = 2000
		self.timer.start(self.Period)

		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient-2','b0RemoteApiAddOn')
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]
		self.base = self.client.simxGetObjectHandle('UR3', self.client.simxServiceCall())[1]
		self.biela = self.client.simxGetObjectHandle('Shape5', self.client.simxServiceCall())[1]
		self.pinza = self.client.simxGetObjectHandle('Camera_hand', self.client.simxServiceCall())[1]

		self.rodMachine.start()

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True

	@QtCore.Slot()
	def detectCircle(self):
		# image is a structure that contains cameraID, width, height, focalx, focaly, alivetime, TypeImage image
		image = self.camerargbdsimple_proxy.getImage()
		print("cameraID: ", image.cameraID)
		'''print("size: ", img.shape)
		print("width: ", image.width)
		print("height: ", image.height)'''
		#print("original: ", image.image.shape)
		img = np.fromstring(image.image, np.uint8).reshape(image.height, image.width, 3)
		print("img: ", img.shape)
		#img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		cv2.imshow("Camera_hand", img)
		print("despues show: ", img.shape)
		cv2.drawMarker(img, (int(image.width), int(image.height)),  (0, 0, 255), cv2.MARKER_CROSS, 100, 1)
		#print("Width: ", int(image.width/2), " Height: ", int(image.height/2))
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# change image to grey and call HoughCircles
		grayImage     = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		circlesImage  = cv2.HoughCircles(grayImage, cv2.HOUGH_GRADIENT, 1.2, 100)		
		if circlesImage is not None:
			for (x, y, r) in circlesImage[0]:
				x = math.ceil(x)
				y = math.ceil(y)
				r = math.ceil(r)
				self.keypoint = [x, y]
				cv2.circle(img, (x, y), r, (255, 255, 255), 4)
				cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
				#cv2.imshow("Camera_hand", img)
				#cv2.waitKey(5)
			return True
		else:
			return False


	@QtCore.Slot()
	def moveArm(self):

		depth_ = self.camerargbdsimple_proxy.getDepth()
		self.depth = np.frombuffer(depth_.depth, dtype=np.float32).reshape(depth_.height, depth_.width)
		ki = self.keypoint[0] - 320
		kj = 240 - self.keypoint[1]
		pdepth = float(self.depth[self.keypoint[0]][self.keypoint[1]])

		if pdepth < 10000 and pdepth > 0:
			self.keypoint.append(pdepth)
			self.keypoint[0] = ki * self.keypoint[2] / 462
			self.keypoint[1] = kj * self.keypoint[2] / 462
			movement = self.client.simxSetObjectPosition(self.target, self.target, self.keypoint, self.client.simxServiceCall())		
		return True

# =============== Slots methods for State Machine ===================
# ===================================================================
# =============== Slots methods for State Machine ===================
# ===================================================================
	#
	# sm_initialize
	#
	@QtCore.Slot()
	def sm_initialize(self):
		print("Entered state initialize")
		self.t_initialize_to_detectCircle.emit()
		pass

	#
	# sm_detectCircle
	#
	@QtCore.Slot()
	def sm_detectCircle(self):
		print("Entered state detectCircle")
		while(True):
			if self.detectCircle():
				break
		self.t_detectCircle_to_moveArm.emit()

		pass

	#
	# sm_dropRod
	#
	@QtCore.Slot()
	def sm_dropRod(self):
		print("Entered state dropRod")
		self.moveArm()
		self.t_dropRod_to_finalize.emit()
		pass

	#
	# sm_moveArm
	#
	@QtCore.Slot()
	def sm_moveArm(self):
		print("Entered state moveArm")
		self.t_moveArm_to_takeRod.emit()
		pass

	#
	# sm_moveRod
	#
	@QtCore.Slot()
	def sm_moveRod(self):
		print("Entered state moveRod")
		self.t_moveRod_to_dropRod.emit()
		pass

	#
	# sm_takeRod
	#
	@QtCore.Slot()
	def sm_takeRod(self):
		print("Entered state takeRod")
		self.t_takeRod_to_moveRod.emit()
		pass

	#
	# sm_finalize
	#
	@QtCore.Slot()
	def sm_finalize(self):
		print("Entered state finalize")
		self.kill.emit()
		pass



# =================================================================
# =================================================================

