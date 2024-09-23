#!/usr/bin/env python3.8

"""
  @author Simone Cottiga
  @email simone.cottiga@phd.units.it
  @email smcotti@gmail.com
"""

import numpy as np
import math
# import pandas as pd


def calcMaxSlip(a,r):
    r = abs(r)
    k1 = 0.02
    k2 = 0.35
    min1 = 0
    min2 = 0.05

    s = k1 * max(a-min1,0) + k2 * max(r-min2,0)
    return s


def slipFunction(input_array):
    input_array = input_array[0]

    output_array = np.empty(int(np.size(input_array)/3*2))

    for i in range(int(np.size(input_array)/3)):
        # extract inputs
        alpha = abs(input_array[i*3])
        beta = input_array[i*3+1]
        rho = input_array[i*3+2]

        # uphill (maximum) slip
        upSlip = calcMaxSlip(alpha,rho)

        # longitudinal slip & velocity
        cLong = upSlip
        kLong = -cLong / 135
        slipLong = kLong * abs(beta) + cLong

        velLong = rho
        if slipLong > 0:
            velLong = rho * (1 - slipLong)
        elif slipLong < 0:
            velLong = rho / (1 + slipLong)

        # lateral slip & velocity
        maxSlipLat = upSlip * 0.5
        c1Lat = 0
        k1Lat = maxSlipLat / 90
        c2Lat = 2 * maxSlipLat
        k2Lat = - maxSlipLat / 90
        if abs(beta) <= 90:
            slipLat = k1Lat * abs(beta) + c1Lat
        else:
            slipLat = k2Lat * abs(beta) + c2Lat

        velLat = rho * slipLat

        magnitude = math.sqrt(velLong**2 + velLat**2)
        deviation = math.degrees(math.atan2(velLat, velLong))

        orientation = beta + math.copysign(deviation, beta)

        output_array[i*2] = orientation
        output_array[i*2+1] = magnitude

    return output_array


# test code. If used, add "import pandas as pd" for csv read/write
if False:
    csv_load_name = '/home/ros/archimede_tilted_floor_tests/simpleSlip_doe_dataset.csv'
    data = pd.read_csv(csv_load_name,header=None)

    inputs = [None] * int(data[0].size*3)

    for i in range(data[0].size):
        inputs[i*3] = (data[0][i])
        inputs[i*3+1] = (data[1][i])
        inputs[i*3+2] = (data[2][i])

    inputs = np.asarray(inputs)
    inputs = inputs[np.newaxis, :]

    outputs = slipFunction(inputs)

    csv_save_name = '/home/ros/archimede_tilted_floor_tests/simpleSlip_doe_dataset_python_outputs.csv'
    np.savetxt(csv_save_name,outputs,delimiter=',')

