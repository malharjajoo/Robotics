import matplotlib.pyplot as plt
import re

# regex: ([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)
# use matching groups

logFile = open('log3.txt', 'r')
contents = logFile.read()

# process log file with regex
listTime = []
listAngle0 = []
listRefAngle0 = []
listAngle1 = []
listRefAngle1 = []
for match in re.finditer('([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)', contents):
	listTime.append(match.group(1))
	listRefAngle0.append(match.group(2))
	listAngle0.append(match.group(3))
	listRefAngle1.append(match.group(4))
	listAngle1.append(match.group(5))

# setup graph titles and axes
fig = plt.gcf();
fig.canvas.set_window_title('after_tuning_-_motor_0')
plt.title('motor[0]')
plt.xlabel('time /s')
plt.ylabel('angle')

# plot graph for motor 0
lineAngle0 = plt.plot(listTime, listAngle0)
lineRefAngle0 = plt.plot(listTime, listRefAngle0)
plt.setp(lineAngle0, color='r', linewidth=1, label = "angle[0]")
plt.setp(lineRefAngle0, color='b', linewidth=1, label = "reference[0]")
plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
plt.axis([
	float(listTime[0]),
	float(listTime[-1]),
	float(min(listAngle0 + listRefAngle0)),
	float(max(listAngle0 + listRefAngle0))
	])
plt.show()

# setup graph title
fig = plt.gcf();
fig.canvas.set_window_title('after_tuning_-_motor_1')
plt.title('motor[1]')

# plot graph for motor 1
lineAngle1 = plt.plot(listTime, listAngle1)
lineRefAngle1 = plt.plot(listTime, listRefAngle1)
plt.setp(lineAngle1, color='#1E8449', linewidth=1, label = "angle[1]")
plt.setp(lineRefAngle1, color='b', linewidth=1, label = "reference[1]")
plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
plt.axis([
	float(listTime[0]),
	float(listTime[-1]),
	float(min(listAngle1 + listRefAngle1)),
	float(max(listAngle1 + listRefAngle1))
	])
plt.show()

logFile.close()
