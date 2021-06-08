import pybullet as p
import math
import numpy as np
import time

class PointCloud:
	"""
	PointCloud: Synthetic point cloud generator based on fake camera location
	"""
	def __init__(self):
		# TODO
		self.file_dir = 'pc.npy'
		# larger step size results sparse point cloud
		self.stepX = 1
		self.stepY = 1

		# depth distortion correction params
		self.farPlane = 10000

		# depth threshold
		self.depth_threshold_x = 4 # m
		self.depth_threshold_y = 4 # m
		self.depth_threshold_z = 4 # m
		self.pointcloud = []

	def save(self, save_dir='pc.npy'):
		print('here')
		if len(self.pointcloud) == 0:
			print('No pointcloud tp save.')
			return

		np.save(open(save_dir, 'wb'), self.pointcloud)

		print('saved at ', save_dir)


	def getRayFromTo(self, mouseX, mouseY):
		width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
		camPos = [camTarget[0] - dist * camForward[0],
					camTarget[1] - dist * camForward[1],
					camTarget[2] - dist * camForward[2]
				 ]


		rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
		lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
		                 rayForward[2] * rayForward[2])
		invLen = self.farPlane * 1. / lenFwd
		rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
		rayFrom = camPos
		oneOverWidth = float(1) / float(width)
		oneOverHeight = float(1) / float(height)

		dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
		dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
		rayToCenter = [
		  rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
		]
		ortho = [
		  -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
		  -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
		  -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
		]

		rayTo = [
		  rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
		  rayFrom[2] + rayForward[2] + ortho[2]
		]
		lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
		alpha = math.atan(lenOrtho / self.farPlane)
		return rayFrom, rayTo, alpha

	def setCamera(self, cameraDistance=0.3, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=[0,0.1,0]):
		p.resetDebugVisualizerCamera(cameraDistance=cameraDistance,
									cameraYaw=cameraYaw,
									cameraPitch=cameraPitch,
									cameraTargetPosition=cameraTargetPosition)
		p.stepSimulation() # this is required to update the scene
		time.sleep(1)

	def generatePointCloud(self, verbose=False):
		width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
		camPos = [
			camTarget[0] - dist * camForward[0],
			camTarget[1] - dist * camForward[1],
			camTarget[2] - dist * camForward[2] 
			]

		imgW = int(width / self.stepX)
		imgH = int(height / self.stepY)

		img = p.getCameraImage(imgW, imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
		rgbBuffer = img[2]
		depthBuffer = img[3]


		if verbose:
			print('camTarget: ', camTarget)
			print("rgbBuffer.shape=", rgbBuffer.shape)
			print("depthBuffer.shape=", depthBuffer.shape)

		self.pointcloud = []
		for w in range(0, imgW, self.stepX):
			for h in range(0, imgH, self.stepY):
				rayFrom, rayTo, alpha = self.getRayFromTo(w * (width / imgW), h * (height / imgH))
				rf = np.array(rayFrom)
				rt = np.array(rayTo)
				vec = rt - rf
				l = np.sqrt(np.dot(vec, vec))
				depthImg = float(depthBuffer[h, w])
				far = 1000.
				near = 0.01
				depth = far * near / (far - (far - near) * depthImg)
				depth /= math.cos(alpha)
				newTo = (depth / l) * vec + rf
				#print(newTo)
				if ( abs(newTo[0]) < self.depth_threshold_x) and (abs(newTo[1]) < self.depth_threshold_y ) and (abs(newTo[2]) < self.depth_threshold_z):
					self.pointcloud.append(newTo)

		self.pointcloud = np.array(self.pointcloud)
		if verbose:
			print('PointCloud size: ', self.pointcloud.shape)
		return self.pointcloud






