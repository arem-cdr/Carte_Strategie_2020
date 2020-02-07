# rappel : "python -m serial.tools.list_ports -v" pour lister les ports com disponibles, et remplacer "com8" ligne 14 par le port com dispo

import serial
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
from math import *

# init serial
ser = serial.Serial('com8', 115200)
L = []
# Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

# Define a top-level widget to hold everything
win = QtGui.QWidget()
win.setWindowTitle("Interface Debug")

plot = pg.PlotWidget()

# creation de la legende
legend = pg.LegendItem(size=(100, 50), offset=(40, 10))
legend.setParentItem(plot.graphicsItem())
# 'pos': (current, current2), 'size': 10, 'pen': None, 'brush': 'g', 'symbol': 'o'
legend.addItem(item=pg.ScatterPlotItem(pen=None, size=10, brush='g', symbol='o'),
               name="Obstacles")
legend.addItem(item=pg.ScatterPlotItem(pen=None, size=35, brush='r', symbol='o'),
               name="Robot")

plot.getPlotItem().setTitle("Pathfinding")
layout = QtGui.QGridLayout()
win.setLayout(layout)

# creation des recepteurs de donnees
DataObstacles = pg.ScatterPlotItem(pen=None, symbol='o',
                                   symbolPen=None, symbolBrush='r')
DataPositionRobot = pg.ScatterPlotItem(pen=None, symbol='o',
                                       symbolPen=None, symbolBrush='r')
DataTargetRobot = pg.ScatterPlotItem(pen=None, symbol='o',
                                     symbolPen=None, symbolBrush='r')

plot.addItem(DataObstacles)
plot.addItem(DataPositionRobot)
plot.addItem(DataTargetRobot)

plot.setXRange(0, 300)  # dimensions de la taille
plot.setYRange(0, 200)

layout.addWidget(plot)  # plot goes on right side, spanning 3 rows
win.show()

# variables diverses sur le robot
positionX = 0
positionY = 0
targetX = 0
targetY = 0
timeElapsedSinceBoot = 0
printingInfo = False

spots = []  # tableau utilisé pour l'affichage d'éléments

while True:
    ser_bytes = ser.readline()
    decoded_byte = 0
    try:
        decoded_byte = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
    except ValueError:
        if printingInfo:
            print("Failed to convert string to float")
    L.insert(0, decoded_byte)
    if (decoded_byte == -1.0):  # -1 : fin d'une séquence

        if printingInfo:
            print(" ")
            print("debut sequence reception")
            print(L)

        State = "INIT"
        linesRRT = []
        linesPath = []

        while len(L) > 0:
            # machine à etat sur la variable State
            if State == "INIT":
                current = L.pop()
                # print(current)
                if current < 0:
                    if current == -2.0:
                        State = "READING CURRENT POSITION"
                        if printingInfo:
                            print("reading current position")
                    if current == -3.0:
                        State = "READING CURRENT TARGET"
                        if printingInfo:
                            print("reading current target")
                    if current == -4.0:
                        State = "READING TIME"
                        if printingInfo:
                            print("reading time")
                    elif current == -101.0:
                        State = "READING OBSTACLES"
                        if printingInfo:
                            print("reading obstacles")
                        spots = []
                    elif current == -102.0:
                        State = "READING RRT BRANCHES"
                        linesRRT.clear()
                        if printingInfo:
                            print("reading RRT branches")
                    elif current == -103.0:
                        State = "READING NODE COUNT"
                        if printingInfo:
                            print("reading node count")
                    elif current == -104.0:
                        State = "READING PATH"
                        linesPath.clear()
                        if printingInfo:
                            print("reading path")
            elif State == "READING CURRENT POSITION":
                current = L.pop()
                # print(current)
                if current < 0 or len(L) == 0:
                    if current == -1:  # fin de la séquence
                        State = "INIT"
                        if printingInfo:
                            print("init state")
                        # affichage du contenu de spots sur le plot
                        DataPositionRobot.setData(spots)
                        spots.clear()
                    else:
                        print("erreur dans reading position. ID error : 1")
                else:
                    current2 = L.pop()
                    # print(current2)
                    if current2 < 0:
                        print("erreur dans reading position. ID error : 2")
                        State = "INIT"
                        L.append(current2)
                    else:
                        positionX = current
                        positionY = current2
                        spots.append(
                            {'pos': (current, current2), 'size': 35, 'pen': None, 'brush': 'r', 'symbol': 'o'})
            elif State == "READING CURRENT TARGET":
                current = L.pop()
                # print(current)
                if current < 0 or len(L) == 0:
                    if current == -1:  # fin de la séquence
                        State = "INIT"
                        if printingInfo:
                            print("init state")
                        DataTargetRobot.setData(spots)
                        spots.clear()
                    else:
                        print("erreur dans reading target. ID error : 1")
                else:
                    current2 = L.pop()
                    # print(current2)
                    if current2 < 0:
                        print("erreur dans reading target. ID error : 2")
                        State = "INIT"
                        L.append(current2)
                    else:
                        targetX = current
                        targetY = current2
            elif State == "READING TIME":
                current = L.pop()
                # print(current)
                if current < 0 or len(L) == 0:
                    if current == -1:  # fin de la séquence
                        State = "INIT"
                        if printingInfo:
                            print("init state")
                    else:
                        print("erreur dans reading time")
                else:
                    timeElapsedSinceBoot = current
                    if printingInfo:
                        print("current time : ", timeElapsedSinceBoot)
            elif State == "READING OBSTACLES":
                current = L.pop()
                # print(current)
                if current < 0 or len(L) == 0:
                    if current == -1:  # fin de la séquence
                        State = "INIT"
                        if printingInfo:
                            print("init state")
                        # affichage du contenu de spots sur le plot
                        DataObstacles.setData(spots)
                        spots.clear()
                    else:
                        print("erreur dans reading obstacles. ID error : 1")
                else:
                    current2 = L.pop()
                    # print(current2)
                    if current2 < 0:
                        print("erreur dans reading obstacles. ID error : 2")
                        State = "INIT"
                        L.append(current2)
                    else:
                        spots.append(
                            {'pos': (current, current2), 'size': 10, 'pen': None, 'brush': 'g', 'symbol': 'o'})
            elif State == "READING RRT BRANCHES":
                current = L.pop()
                # print(current)
                if current == -1:  # fin de la séquence
                    State = "INIT"
                    if printingInfo:
                        print("init state")
                else:
                    if len(L) < 3:
                        print("erreur dans reading RRT branches. ID error : 1")
                        State = "INIT"
                        L.clear()
                    else:
                        current2 = L.pop()
                        if current2 < 0:
                            State = "INIT"
                            L.append(current2)
                        else:
                            current3 = L.pop()
                            if current3 < 0:
                                State = "INIT"
                                L.append(current3)
                            else:
                                current4 = L.pop()
                                if current4 < 0:
                                    State = "INIT"
                                    L.append(current4)
                                else:
                                    # print(current2)
                                    # print(current3)
                                    # print(current4)
                                    linesRRT.append(pg.LineSegmentROI(
                                        [[current, current2], [current3, current4]], pen=pg.mkPen('g', width=1)))
            elif State == "READING NODE COUNT":
                current = L.pop()
                if current < 0 or len(L) == 0:
                    if current == -1:  # fin de la séquence
                        State = "INIT"
                        if printingInfo:
                            print("init state")
                    else:
                        print("erreur dans reading node count")
                else:
                    print("node count : ", current)
            elif State == "READING PATH":
                current = L.pop()
                # print(current)
                if current == -1:  # fin de la séquence
                    State = "INIT"
                    if printingInfo:
                        print("init state")
                else:
                    if len(L) < 3:
                        print("erreur dans reading Path. ID error : 1")
                        State = "INIT"
                        L.clear()
                    else:
                        current2 = L.pop()
                        if current2 < 0:
                            State = "INIT"
                            L.append(current2)
                        else:
                            current3 = L.pop()
                            if current3 < 0:
                                State = "INIT"
                                L.append(current3)
                            else:
                                current4 = L.pop()
                                if current4 < 0:
                                    State = "INIT"
                                    L.append(current4)
                                else:
                                    # print(current2)
                                    # print(current3)
                                    # print(current4)
                                    linesPath.append(pg.LineSegmentROI(
                                        [[current, current2], [current3, current4]], pen=pg.mkPen('r', width=1)))

        plot.clear()

        for i in range(0, len(linesPath)):
            plot.addItem(linesPath[i])
        plot.addItem(DataObstacles)
        plot.addItem(DataPositionRobot)
        plot.addItem(DataTargetRobot)
        for i in range(0, len(linesRRT)):
            plot.addItem(linesRRT[i])

        L.clear()  # au cas où L ne soit pas vide, normalement c'est deja le cas
    QtGui.QApplication.processEvents()

QtGui.QApplication.processEvents()
app.exec_()
