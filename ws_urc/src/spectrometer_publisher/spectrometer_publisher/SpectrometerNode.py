#!/usr/bin/env python3

import cv2
import numpy as np
from math import factorial

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from custom_interfaces.msg import Spectrometer


def savitzky_golay(y, window_size, order, deriv=0, rate=1):
	"""
	Savitzky-Golay filter implementation
	From: https://scipy.github.io/old-wiki/pages/Cookbook/SavitzkyGolay
	"""
	try:
		window_size = np.abs(np.int32(window_size))
		order = np.abs(np.int32(order))
	except ValueError:
		raise ValueError("window_size and order have to be of type int")
	if window_size % 2 != 1 or window_size < 1:
		raise TypeError("window_size size must be a positive odd number")
	if window_size < order + 2:
		raise TypeError("window_size is too small for the polynomials order")
	order_range = range(order+1)
	half_window = (window_size -1) // 2
	# precompute coefficients
	b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
	m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
	# pad the signal at the extremes with values taken from the signal itself
	firstvals = y[0] - np.abs(y[1:half_window+1][::-1] - y[0])
	lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
	y = np.concatenate((firstvals, y, lastvals))
	return np.convolve(m[::-1], y, mode='valid')


def readcal(width):
	"""
	Read and process calibration data
	"""
	errors = 0
	try:
		print("Loading calibration data...")
		file = open('config/caldata.txt', 'r')
	except:
		errors = 1

	try:
		lines = file.readlines()
		line0 = lines[0].strip()
		pixels = line0.split(',')
		pixels = [int(i) for i in pixels]
		line1 = lines[1].strip()
		wavelengths = line1.split(',')
		wavelengths = [float(i) for i in wavelengths]
	except:
		errors = 1

	try:
		if (len(pixels) != len(wavelengths)):
			errors = 1
		if (len(pixels) < 3):
			errors = 1
		if (len(wavelengths) < 3):
			errors = 1
	except:
		errors = 1

	if errors == 1:
		print("Loading of Calibration data failed (missing caldata.txt or corrupted data!")
		print("Loading placeholder data...")
		print("You MUST perform a Calibration to use this software!\n\n")
		pixels = [0, 400, 800]
		wavelengths = [380, 560, 750]

	wavelengthData = []

	if (len(pixels) == 3):
		print("Calculating second order polynomial...")
		coefficients = np.poly1d(np.polyfit(pixels, wavelengths, 2))
		print(coefficients)
		C1 = coefficients[2]
		C2 = coefficients[1]
		C3 = coefficients[0]
		print("Generating Wavelength Data!\n\n")
		for pixel in range(width):
			wavelength = ((C1*pixel**2)+(C2*pixel)+C3)
			wavelength = round(wavelength, 6)
			wavelengthData.append(wavelength)
		print("Done! Note that calibration with only 3 wavelengths will not be accurate!")
		if errors == 1:
			message = 0
		else:
			message = 1

	if (len(pixels) > 3):
		print("Calculating third order polynomial...")
		coefficients = np.poly1d(np.polyfit(pixels, wavelengths, 3))
		print(coefficients)
		C1 = coefficients[3]
		C2 = coefficients[2]
		C3 = coefficients[1]
		C4 = coefficients[0]
		print("Generating Wavelength Data!\n\n")
		for pixel in range(width):
			wavelength = ((C1*pixel**3)+(C2*pixel**2)+(C3*pixel)+C4)
			wavelength = round(wavelength, 6)
			wavelengthData.append(wavelength)

		predicted = []
		for i in pixels:
			px = i
			y = ((C1*px**3)+(C2*px**2)+(C3*px)+C4)
			predicted.append(y)

		corr_matrix = np.corrcoef(wavelengths, predicted)
		corr = corr_matrix[0, 1]
		R_sq = corr**2
		print("R-Squared="+str(R_sq))
		message = 2

	if message == 0:
		calmsg1 = "UNCALIBRATED!"
		calmsg2 = "Defaults loaded"
		calmsg3 = "Perform Calibration!"
	if message == 1:
		calmsg1 = "Calibrated!!"
		calmsg2 = "Using 3 cal points"
		calmsg3 = "2nd Order Polyfit"
	if message == 2:
		calmsg1 = "Calibrated!!!"
		calmsg2 = "Using > 3 cal points"
		calmsg3 = "3rd Order Polyfit"

	returndata = []
	returndata.append(wavelengthData)
	returndata.append(calmsg1)
	returndata.append(calmsg2)
	returndata.append(calmsg3)
	return returndata


class SpectrometerNode(Node):

	def __init__(self):
		super().__init__('spectrometer_node')
		
		# Declare ROS2 parameters with defaults
		# second argument is default value
		self.declare_parameter('device', 0, ParameterDescriptor(description='Video Device number'))
		self.declare_parameter('fps', 30, ParameterDescriptor(description='Frame Rate'))
		self.declare_parameter('frame_width', 800, ParameterDescriptor(description='Frame width'))
		self.declare_parameter('frame_height', 600, ParameterDescriptor(description='Frame height'))
		self.declare_parameter('timer_period', 0.033, ParameterDescriptor(description='Timer period in seconds (~30Hz)'))
		
		# Get parameters
		dev = self.get_parameter('device').value
		fps = self.get_parameter('fps').value
		self.frameWidth = self.get_parameter('frame_width').value
		self.frameHeight = self.get_parameter('frame_height').value
		timer_period = self.get_parameter('timer_period').value
		
		# Initialize video capture
		self.cap = cv2.VideoCapture(f'/dev/video{dev}', cv2.CAP_V4L)
		self.get_logger().info("[info] W, H, FPS")
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frameWidth)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frameHeight)
		self.cap.set(cv2.CAP_PROP_FPS, fps)
		self.get_logger().info(f"Width: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
		self.get_logger().info(f"Height: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
		self.get_logger().info(f"FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
		
		# Peak detection settings
		self.savpoly = 7
		
		# Initialize intensity array
		self.intensity = [0] * self.frameWidth
		
		# Load calibration data
		caldata = readcal(self.frameWidth)
		self.wavelengthData = caldata[0]
		self.calmsg1 = caldata[1]
		self.calmsg2 = caldata[2]
		self.calmsg3 = caldata[3]
		
		# Create ROS2 publisher
		self.publisher_ = self.create_publisher(Spectrometer, 'spectrometer_data', 10)
		
		# Create timer for periodic spectrometer data capture and publishing
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		self.get_logger().info('Spectrometer Node has been started.')
	
	def timer_callback(self):
		if not self.cap.isOpened():
			self.get_logger().error('Camera is not opened')
			return
		
		ret, frame = self.cap.read()
		
		if not ret:
			self.get_logger().warning('Failed to capture frame')
			return
		
		# Crop the frame
		y = int((self.frameHeight / 2) - 40)
		x = 0
		h = 80
		w = self.frameWidth
		cropped = frame[y:y+h, x:x+w]
		bwimage = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
		rows, cols = bwimage.shape
		halfway = int(rows / 2)
		
		# Process intensity data
		for i in range(cols):
			dataminus1 = bwimage[halfway-1, i]
			datazero = bwimage[halfway, i]
			dataplus1 = bwimage[halfway+1, i]
			data = (int(dataminus1) + int(datazero) + int(dataplus1)) / 3
			data = np.uint8(data)
			self.intensity[i] = data
		
		# Apply Savitzky-Golay filter to intensity data
		self.intensity = np.array(self.intensity)
		self.intensity = savitzky_golay(self.intensity, 17, self.savpoly)
		self.intensity = self.intensity.astype(int)
		
		# Publish spectrometer data to ROS2 topic
		msg = Spectrometer()
		msg.wavelengths = [float(w) for w in self.wavelengthData]
		msg.intensity = [int(i) for i in self.intensity]
		self.publisher_.publish(msg)
	
	def destroy_node(self):
		"""Clean up resources before shutting down"""
		if self.cap.isOpened():
			self.cap.release()
		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)

	spectrometer_node = SpectrometerNode()

	try:
		rclpy.spin(spectrometer_node)
	except KeyboardInterrupt:
		pass

	spectrometer_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()





		