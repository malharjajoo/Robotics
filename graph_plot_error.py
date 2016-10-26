import matplotlib.pyplot as plt
import re

# regex: ([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)
# use matching groups

logFile = open('log2.txt', 'r')
contents = logFile.read()

# process log file with regex
listTime = []
listError0 = []
listError1 = []
for match in re.finditer('([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)\t([0-9.]+)', contents):
	listTime.append(match.group(1))
	listError0.append(float(match.group(2)) - float(match.group(3)))
	listError1.append(float(match.group(4)) - float(match.group(5)))
# setup graph titles and axes
fig = plt.gcf();
fig.canvas.set_window_title('after_tuning_-_motor_0_error')
plt.title('motor[0]')
plt.xlabel('time /s')
plt.ylabel('difference')

# plot error graph for motor 0
lineError0 = plt.plot(listTime, listError0)
plt.setp(lineError0, color='r', linewidth=1, label = "error[0]")
plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
plt.axis([
	float(listTime[0]),
	float(listTime[-1]),
	float(min(listError0)),
	float(max(listError0))
	])
plt.show()

# setup graph title
fig = plt.gcf();
fig.canvas.set_window_title('after_tuning_-_motor_1_error')
plt.title('motor[1]')

# plot error graph for motor 1
lineError1 = plt.plot(listTime, listError1)
plt.setp(lineError1, color='#1E8449', linewidth=1, label = "error[1]")
plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
plt.axis([
	float(listTime[0]),
	float(listTime[-1]),
	float(min(listError1)),
	float(max(listError1))
	])
plt.show()



logFile.close()
