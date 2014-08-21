import kinect_ext

k = kinect_ext.Kinect()
k.init()

while 1:
	k.update()
	if k.bodies:
		print k.bodies[0].joints



# data viz dedicated to demystifying public local and natl gov data sets
# way to ma relationships, graph campaign contributions

#tools
#coveritlive
#buffer 
#google fusion tables
#datawrapper
#irc