import kinect_ext

k = kinect_ext.Kinect()
k.init()

while 1:
	k.update()
	if k.bodies():
		print k.bodies()[0].joints()