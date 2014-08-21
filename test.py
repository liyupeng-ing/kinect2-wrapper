import kinect_ext

k = kinect_ext.Kinect()
k.Init()

while 1:
	k.Update()

	if k.Bodies:
		print k.Bodies[0].HandRightState



# data viz dedicated to demystifying public local and natl gov data sets
# way to ma relationships, graph campaign contributions

#tools
#coveritlive
#buffer 
#google fusion tables
#datawrapper
#irc