from math import pi
from time import sleep

import numpy as np
import matplotlib.pyplot as plt
import csv

import numpy.random

while 1:
    with open('data.csv', 'r') as dataFile:
        Line_reader = csv.reader(dataFile, delimiter=',')

        theta = []
        radius = []
        size = []

        sizes = numpy.random.rand(3)

        for detectedObject in Line_reader:
            theta.append((float)(detectedObject[0]) * (pi / 180))
            radius.append(detectedObject[1])
            size.append(detectedObject[2])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')
        c = ax.scatter(theta, radius, s=size)
        plt.show()
        sleep(40)
