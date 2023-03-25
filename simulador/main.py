import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt

plt.ion()

car = cp. CarCoppelia()

car.startMission()


t0 = car.getTime()

while (car.getTime() - t0) < 50.0:
	
	car.step()
	
	car.setSteer(30.0*np.sin(0.5*car.getTime()))
	car.setVel(np.abs(np.sin(0.5*car.getTime())))
	
	# camera
	frame = car.getImage()
	image_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
	print('image...')
	
	plt.figure(1)
	plt.gca().imshow(image_bgr, origin='lower')
	plt.show()
	plt.pause(0.01)

