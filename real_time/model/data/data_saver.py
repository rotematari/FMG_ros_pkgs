from os import listdir
from os.path import isfile, join
import os 
# Get the current directory of the script being run
current_directory = os.path.dirname(os.path.realpath(__file__))

# Navigate up  directori
parent_directory = os.path.abspath(os.path.join(current_directory, os.pardir))
# Change the working directory
os.chdir(parent_directory)

import numpy as np
import serial
import time as t

from NatNetClient import NatNetClient

import matplotlib.pyplot as plt 
import yaml
import argparse
import pandas as pd

# from FMG_project.utils.utils import print_not_numeric_vals
data_dir = r'data/data'

with open(r'config.yaml', 'r') as f:
    args = yaml.safe_load(f)

config = argparse.Namespace(**args)
# dirs = [f for f in listdir(dirpath)]
# make sure the 'COM#' is set according the Windows Device Manager /dev/ttyACM0
ser = serial.Serial('COM3', 115200)

def is_not_numeric(x):
    try:
        float(x)
        return False
    except ValueError:
        return True
    

def print_not_numeric_vals(df):

    mask = df.drop(['sesion_time_stamp'],axis=1).applymap(is_not_numeric)
    non_numeric_values = df[mask].stack().dropna()
    print(non_numeric_values)

    return non_numeric_values


# print format: t,Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz,F1,F2,F3,F4,B1,B2,S1,S2,S3,S4,class
# make new file names and locate them in the correct directory
def write_first_line(f):
 
    f.write("S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14,S15,S16,S17,S18,S19,S20,S21,S22,S23,S24,S25,S26,S27,S28,S29,S30,S31,S32,")
    # f.write("S21,S22,S23,S24,S25,S26,S27,S28,S29,S30,S31,S32")
    f.write("M1x,M1y,M1z,M2x,M2y,M2z,M3x,M3y,M3z,M4x,M4y,M4z,")
    f.write("sesion_time_stamp,\n")


def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    print( "Received frame for rigid body", id )

def receiveRigidBodyList( rigidBodyList, stamp ):
    for (ac_id, pos, quat, valid) in rigidBodyList:
        # print("rigidBodyList")
        # print(rigidBodyList)
        # print(type(rigidBodyList))
        # print(len(rigidBodyList))
        if not valid:
            # skip if rigid body is not valid
            continue
        
        # print('id: ', ac_id, 'pos:', pos, 'quat:', quat) 


def init_natnetClient():
           
    # start natnet interface
    natnet = NatNetClient(rigidBodyListListener=receiveRigidBodyList,server="132.66.51.232")#rigidBodyListListener=receiveRigidBodyList)

    keys = ['chest', 'shoulder', 'elbow', 'wrist']
    chest = 1
    shoulder = 2
    elbow = 3
    wrist = 4
    return natnet

    # This dictionary matches the rigid body id (key) to it's name (value)
    motive_matcher = {chest: 'chest',
                        shoulder: 'shoulder',
                        shoulder: 'elbow',
                        elbow: 'wrist',}


def write_line(f,marker_data,sesion_time_stamp):

    # sesion_time_stamp = t.strftime("%d_%b_%Y_%H:%M", t.gmtime())


    line = ser.readline()  # read a byte
    sensor_string = line.decode('utf-8')  # ('latin-1')  # convert the byte string to a unicode string
    sensor_string = sensor_string.strip()
    sensor_string.replace("'", '')
    sensor_string.replace("[", '')
    sensor_string.replace("]", '')


    #  # test 
    # sensor_string = ''.join(str(i)+',' for i in range(48))

    marker_string =[]
    for i in range(len(marker_data)):

        marker_string += [str(j)for j in marker_data[i][1]] 

    marker_string = ''.join(str(s)+',' for s in marker_string)

    # sensor_string , marker_string , sesion_time_stamp
    f.write(f'{sensor_string}' +f'{marker_string}'+ f'{sesion_time_stamp}' + '\n')


    # return sesion_time_stamp


def plot_data(config,data):
    fig, (ax1, ax2) = plt.subplots(1, 2,figsize=(40,5))

    ax1.plot(data.drop(['sesion_time_stamp'],axis=1)[config.positoin_label_inedx])
    ax2.plot(data.drop(['sesion_time_stamp'],axis=1)[config.fmg_index])
    ax1.legend()

    plt.show() 





if __name__ == '__main__':


    for i in range(10):
        ser.readline()

    t_start = t.time()
    sesion_time_stamp = t.strftime("%d_%b_%Y_%H_%M", t.gmtime())
    file_name = sesion_time_stamp + '_movment_2'+'.csv'
    NatNet = init_natnetClient()
    print(file_name)
    f = open(join(data_dir, file_name), "w")

    write_first_line(f)
    NatNet.run()
    t.sleep(5)
    marker_data = NatNet.rigidBodyList

    for i in range(10000):
      
      marker_data = NatNet.rigidBodyList

      write_line(f,sesion_time_stamp=sesion_time_stamp ,marker_data=marker_data)

      if i%100==0:
          print(i)
    
    f.close()
    NatNet.stop()

    t_end = t.time()
    print(t_end-t_start-0.5)
    
    ser.close()
    print("finished")

    ## checks data 
    # df = pd.read_csv(join(data_dir,file_name))
    

    # not_numeric_vals = print_not_numeric_vals(df)

    # if not_numeric_vals.shape[0] == 0:
    #     plot_data(config=config,data=df)

    # else :
    #     print("clean data")


