import sys
import matplotlib.pyplot as plt

readings = ""

with open(sys.argv[1], "r") as voltsfile:
    readings = voltsfile.read()

readings = readings.split("\n")
offset = int(readings[0].split(" ")[0])

times = []
volts = []

for pair in readings:
    pair = pair.split(" ")
    times.append(int(pair[0]) - offset)
    volts.append(float(pair[1]))

plt.scatter(times, volts)
plt.show()
