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
import enum
import b0RemoteApi
import vrep
import cv2
import numpy as np
import math

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel



# THIS NEED TO BE C
class StateMachine(enum.Enum):
	Searching = 1
	Moving = 2
	Taking = 3

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		# connecting to blue-zero client

		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.state_machine = StateMachine.Searching
		self.camera_data = {}
		# if the client connects we take the object handlers
		handlers_list = [self.client.simxGetObjectHandle('target', self.client.simxServiceCall()),
							self.client.simxGetObjectHandle('UR3', self.client.simxServiceCall()),
							self.client.simxGetObjectHandle('Shape5', self.client.simxServiceCall()),
							self.client.simxGetObjectHandle('Camera_hand', self.client.simxServiceCall()),
		]
		if all(map(lambda x: x[0], handlers_list)):
			print("[INFO] connected to all handler components")
			self.target, self.base, self.rod, self.hand_camera = [i[1] for i in handlers_list]
		else:
			raise Exception("[WARNING] COULDN'T CONNECT TO SOME HANDLER COMPONENTS")




	def get_camera_data(self):
		results = self.client.simxGetVisionSensorImage(self.hand_camera, False, self.client.simxServiceCall())
		if len(results) == 3:
			self.camera_data['width'] = results[1][0]
			self.camera_data['height'] = results[1][1] 
			self.camera_data['image'] = np.fromstring(results[2], np.uint8).reshape(self.camera_data['height'], self.camera_data['width'], 3)

		else:
			print("[WARNING] AN ERRROR OCURRED TAKING CAMERA IMAGE DATA: ERROR no {}".format(results[0]))
		
		results = self.client.simxGetVisionSensorDepthBuffer(self.hand_camera,True, True, self.client.simxServiceCall())
		if len(results) == 3:
			self.camera_data['depth'] = np.fromstring(results[2], np.float32).reshape(self.camera_data['height'], self.camera_data['width'], 1)
		else:
			print("[WARNING] AN ERRROR OCURRED TAKING CAMERA DEPTH DATA: ERROR no {}".format(results[0]))
		if not all(map(lambda x: x in self.camera_data, ['image', 'width', 'height', 'depth'])):
			print("[ERROR] NOT ALL")
			return False
		return True

	def detectCircle(self) -> bool:
		# image is a structure that contains cameraID, width, height, focalx, focaly, alivetime, TypeImage image
		
		if not self.get_camera_data():
			return False
		
		#print("original: ", image.image.shape)
		print("img: ", self.camera_data['image'].shape)
		img = cv2.cvtColor(self.camera_data['image'], cv2.COLOR_RGB2BGR)
		cv2.imshow("Camera_hand", self.camera_data['image'])
		
		cv2.drawMarker(img, (self.camera_data['width'], self.camera_data['height']),  (0, 0, 255), cv2.MARKER_CROSS, 100, 1)
		#print("Width: ", int(image.width/2), " Height: ", int(image.height/2))
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# change image to grey and call HoughCircles
		grayImage     = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		circlesImage  = cv2.HoughCircles(grayImage, cv2.HOUGH_GRADIENT, 1.2, 100)
		if circlesImage is not None:
			circlesImage = np.ceil(circlesImage)		
			for (x, y, r) in circlesImage[0]:
				x = math.ceil(x)
				y = math.ceil(y)
				r = math.ceil(r)
				self.keypoint = [x, y]
				cv2.circle(img, (x, y), r, (255, 255, 255), 4)
				cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
				cv2.imshow("Camera_hand", img)
				cv2.waitKey(5)
			return True
		else:
			return False


	def moveArm(self):

		rod_position = self.client.simxGetObjectPosition(self.rod, -1, self.client.simxServiceCall())
		target_position = self.client.simxGetObjectPosition(self.target, -1, self.client.simxServiceCall())
		print("----", rod_position[0], target_position[0])
		if rod_position[0] == target_position[0] == True:
			rod_position = np.array(rod_position[1])
			print("ROD ", rod_position)
			target_position = np.array(target_position[1])
			print("TARGET ", target_position)
			distance = (target_position - rod_position)
			print("DISTANCE: ", distance)
			if not all(map(lambda x: x < 0.01, distance)):
				mov = self.client.simxSetObjectPosition(self.target, -1, list(rod_position), self.client.simxServiceCall())
				return False
			else:
				return True


		# ki = self.keypoint[0] - 320
		# kj = 240 - self.keypoint[1]
		# pdepth = float(self.camera_data['depth'][self.keypoint[0]][self.keypoint[1]])

		# if pdepth < 10000 and pdepth > 0:
		# 	self.keypoint.append(pdepth)
		# 	self.keypoint[0] = ki * self.keypoint[2] / 462
		# 	self.keypoint[1] = kj * self.keypoint[2] / 462
		# 	aa = (int(self.keypoint[0]), int(self.keypoint[1]), int(self.keypoint[2]))
		# 	movement = self.client.simxSetObjectPosition(self.target, self.target, aa, self.client.simxServiceCall())		
		return False

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:

		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True



	def displayImage(self, image, resolution):
		img = np.fromstring(image, np.uint8).reshape( resolution[1],resolution[0], 3)
		img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		cv2.drawMarker(img, (int(resolution[0]/2), int(resolution[1]/2)),  (0, 0, 255), cv2.MARKER_CROSS, 100, 1);
		cv2.imshow(self.cameraName, img)
		cv2.waitKey(1)


	@QtCore.Slot()
	def compute(self):
		if self.state_machine == StateMachine.Searching:
			print("SEARCHING STATE")
			self.state_machine = StateMachine.Moving if self.detectCircle() else StateMachine.Searching
		
		if self.state_machine == StateMachine.Moving:
			print("MOVING ARM STATE")
			self.state_machine = StateMachine.Moving if self.moveArm() else StateMachine.Searching
	
		

		# computeCODE
		# try:
		#   self.differentialrobot_proxy.setSpeedBase(100, 0)
		# except Ice.Exception as e:
		#   traceback.print_exc()
		#   print(e)

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform('rgbd', z, 'laser')
		# r.printvector('d')
		# print(r[0], r[1], r[2])

		return True


