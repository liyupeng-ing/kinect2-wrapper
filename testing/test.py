import kinect_ext

k = kinect_ext.Kinect()
k.init()

while 1:
	k.update()
	print "here"
	if k.bodies():
		print "here"
		print k.bodies()[0].joints()