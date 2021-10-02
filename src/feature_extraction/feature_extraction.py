#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import math
from math import pi
import time
import csv
import os

import numpy as np
import matplotlib.pyplot as plt
#!!!whiten function to normalize whole feature individually

np.set_printoptions(threshold=np.nan) #to print whole array in terminal
#--- open files

#array declaration part
torsoXYZ = [0]*3
torsoArr = []
body_data = [0]*460 #(degOfFree*numberOfJoints+1)*maxUserNumber
max_user = 1
degOfFree = 3
head = [0]*degOfFree*max_user
neck = [0]*degOfFree*max_user
leftShoulder = [0]*degOfFree*max_user
rightShoulder = [0]*degOfFree*max_user
leftElbow = [0]*degOfFree*max_user
rightElbow = [0]*degOfFree*max_user
leftHand = [0]*degOfFree*max_user
rightHand = [0]*degOfFree*max_user
torso = [0]*degOfFree*max_user
leftHip = [0]*degOfFree*max_user
rightHip = [0]*degOfFree*max_user
leftKnee = [0]*degOfFree*max_user
rightKnee = [0]*degOfFree*max_user
leftFoot = [0]*degOfFree*max_user
rightFoot = [0]*degOfFree*max_user
users_id = [0]*max_user
obsFreq = 100  # to decide the frequency of action_recognition_node
featureVector = [0]*10
#--For Training Set
FstElement = []
SndElement = []
TrdElement = []
FthElement = []
FifthElement = []
SthElement = []
SevthElement = []
EthElement = []
NthElement = []
TthElement = []
ElethElement = []

#angle calculation variables
normal_vector1= [0]*3
orientation_vector= [0]*3
hip_center = [0]*3
normOrientationVector = 0

rs_vector1=[0]*3
rs_vector2=[0]*3
norm_rs_vector1=0
norm_rs_vector2=0

re_vector1=[0]*3
re_vector2=[0]*3
norm_re_vector1=0
norm_re_vector2=0

rp_vector1=[0]*3
rp_vector2=[0]*3
norm_rp_vector1=0
norm_rp_vector2=0

pa_vector1=[0]*3
pa_vector2=[0]*3
norm_pa_vector1=0
norm_pa_vector2=0

rightshoulder_angle = 0
rightelbow_angle = 0
rightpit_angle = 0
waist_angle = 0
lhand_distance = 0
profile_angle = 0

#thand relative calculation variables
rhandrelativecoordinate=[0]*3
rhandrelativecoordinate[0]=0.1
rhandrelativecoordinate[1]=0.1
rhandrelativecoordinate[2]=0.1



situation = "NULL"


def initialise():

    rospy.init_node('feature_extraction_node',anonymous=True) #node name
    run() # To prevent the thread from exiting

def run():
    rate=rospy.Rate(obsFreq)
    rospy.Subscriber('openni2_camera_node/bodyJointArr',String,callback) #subscriber defined

    waist_angle_publisher=rospy.Publisher('body_publisher/waist', String, queue_size=10) #Publishers to publish spesific angle data
    rightshoulder_angle_publisher=rospy.Publisher('body_publisher/rightshoulder', String, queue_size=10)
    rightelbow_angle_publisher=rospy.Publisher('body_publisher/rightelbow', String, queue_size=10)
    rightpit_angle_publisher=rospy.Publisher('body_publisher/rightpit', String, queue_size=10)
    lhand_distance_publisher=rospy.Publisher('body_publisher/lhand_distance', String, queue_size=10)
    profile_angle_publisher=rospy.Publisher('body_publisher/profile', String, queue_size=10)
    rhandrelativecoordinate_x_publisher=rospy.Publisher('body_publisher/rhandrelativecoordinate/x', String, queue_size=10)
    rhandrelativecoordinate_y_publisher=rospy.Publisher('body_publisher/rhandrelativecoordinate/y', String, queue_size=10)
    rhandrelativecoordinate_z_publisher=rospy.Publisher('body_publisher/rhandrelativecoordinate/z', String, queue_size=10)
    while not rospy.is_shutdown():
        dataRegulation()
        decide_feature_vectors()

        msgQ.put(str(waist_angle) + " " + situation)

        waist_angle_publisher.publish(str(math.ceil(waist_angle)))
        rightshoulder_angle_publisher.publish(str(math.ceil(rightshoulder_angle)))
        rightelbow_angle_publisher.publish(str(math.ceil(rightelbow_angle)))
        rightpit_angle_publisher.publish(str(math.ceil(rightpit_angle)))
        lhand_distance_publisher.publish(str(math.ceil(lhand_distance)))
        profile_angle_publisher.publish(str(math.ceil(profile_angle)))
        rhandrelativecoordinate_x_publisher.publish(str(abs(rhandrelativecoordinate[0])))
        rhandrelativecoordinate_y_publisher.publish(str(abs(rhandrelativecoordinate[1])))
        rhandrelativecoordinate_z_publisher.publish(str(abs(rhandrelativecoordinate[2])))
        rate.sleep()

def callback(data):
    global body_data
    body_data = map(int, data.data.split(','))

#Data of the joints
def dataRegulation():
    global max_user
    global degOfFree # x ,y ,z
    global body_data

    global head,neck
    global rightShoulder,leftShoulder
    global leftElbow,rightElbow
    global leftHand,rightHand
    global torso,leftHip,rightHip
    global leftKnee,rightKnee
    global leftFoot,rightFoot,users_id


    for i in range(max_user):
        users_id[i]=body_data[46*i]

    if users_id[i]!=0: #if there is no user it will not create array
        head[i*degOfFree]=body_data[i*46+1] #x The order is same for all joints
        head[i*degOfFree+1]=body_data[i*46+2] #y The order is same for all joints
        head[i*degOfFree+2]=body_data[i*46+3] #z  The order is same for all joints

        neck[i*degOfFree]=body_data[i*46+4]
        neck[i*degOfFree+1]=body_data[i*46+5]
        neck[i*degOfFree+2]=body_data[i*46+6]

        leftShoulder[i*degOfFree]=body_data[i*46+7]
        leftShoulder[i*degOfFree+1]=body_data[i*46+8]
        leftShoulder[i*degOfFree+2]=body_data[i*46+9]

        rightShoulder[i*degOfFree]=body_data[i*46+10]
        rightShoulder[i*degOfFree+1]=body_data[i*46+11]
        rightShoulder[i*degOfFree+2]=body_data[i*46+12]

        leftElbow[i*degOfFree]=body_data[i*46+13]
        leftElbow[i*degOfFree+1]=body_data[i*46+14]
        leftElbow[i*degOfFree+2]=body_data[i*46+15]

        rightElbow[i*degOfFree]=body_data[i*46+16]
        rightElbow[i*degOfFree+1]=body_data[i*46+17]
        rightElbow[i*degOfFree+2]=body_data[i*46+18]

        leftHand[i*degOfFree]=body_data[i*46+19]
        leftHand[i*degOfFree+1]=body_data[i*46+20]
        leftHand[i*degOfFree+2]=body_data[i*46+21]

        rightHand[i*degOfFree]=body_data[i*46+22]
        rightHand[i*degOfFree+1]=body_data[i*46+23]
        rightHand[i*degOfFree+2]=body_data[i*46+24]

        torso[i*degOfFree]=body_data[i*46+25]
        torso[i*degOfFree+1]=body_data[i*46+26]
        torso[i*degOfFree+2]=body_data[i*46+27]

        leftHip[i*degOfFree]=body_data[i*46+28]
        leftHip[i*degOfFree+1]=body_data[i*46+29]
        leftHip[i*degOfFree+2]=body_data[i*46+30]

        rightHip[i*degOfFree]=body_data[i*46+31]
        rightHip[i*degOfFree+1]=body_data[i*46+32]
        rightHip[i*degOfFree+2]=body_data[i*46+33]

        leftKnee[i*degOfFree]=body_data[i*46+34]
        leftKnee[i*degOfFree+1]=body_data[i*46+35]
        leftKnee[i*degOfFree+2]=body_data[i*46+36]

        rightKnee[i*degOfFree]=body_data[i*46+37]
        rightKnee[i*degOfFree+1]=body_data[i*46+38]
        rightKnee[i*degOfFree+2]=body_data[i*46+39]

        leftFoot[i*degOfFree]=body_data[i*46+40]
        leftFoot[i*degOfFree+1]=body_data[i*46+41]
        leftFoot[i*degOfFree+2]=body_data[i*46+42]

        rightFoot[i*degOfFree]=body_data[i*46+43]
        rightFoot[i*degOfFree+1]=body_data[i*46+44]
        rightFoot[i*degOfFree+2]=body_data[i*46+45]

##################################################################################
def decide_feature_vectors():
    global head,neck
    global rightShoulder,leftShoulder
    global leftElbow,rightElbow
    global leftHand,rightHand
    global torso,leftHip,rightHip
    global leftKnee,rightKnee
    global leftFoot,rightFoot,users_id

    global rightpit_angle
    global rightelbow_angle
    global rightshoulder_angle
    global waist_angle
    global lhand_distance
    global profile_angle
    global rhandrelativecoordinate

    #This is the normal vector that oriented in y direction all the time
    normal_vector1[0]=0
    normal_vector1[1]=1
    normal_vector1[2]=0
    #Assume leftHip and rightHip is always symetrical, so we can take their average to obtain hip_center
    hip_center[0]=(leftHip[0]+rightHip[0])/2
    hip_center[1]=(leftHip[1]+rightHip[1])/2
    hip_center[2]=(leftHip[2]+rightHip[2])/2
    #This is the vector which goes from hip_center to torso
    orientation_vector[0]=torso[0]-hip_center[0]
    orientation_vector[1]=torso[1]-hip_center[1]
    orientation_vector[2]=torso[2]-hip_center[2]

    #The angle between orientation vector and normal vector will give the waist angle of a person

    #First calculate norm of orientation vector
    normOrientationVector=math.sqrt((orientation_vector[0]*orientation_vector[0])+(orientation_vector[1]*orientation_vector[1])+(orientation_vector[2]*orientation_vector[2]))
    #Calculate dot product of normal_vector1 and orientation_vector
    dot_product=normal_vector1[1]*orientation_vector[1]
    #Now calculate angle between them
    try:
        waist_angle = 180 - (math.acos(float(dot_product) / float(normOrientationVector*1)) * 180/pi)
        global situation
        if waist_angle<45:
                situation = "standing"
        elif waist_angle>=45:
                situation = "leaning forward"
    except ZeroDivisionError:
        pass
    ###################################################################################################### rightshoulder_angle calculations
    rs_vector1[0]=(rightShoulder[0]-torso[0])
    rs_vector1[1]=(rightShoulder[1]-torso[1])
    rs_vector1[2]=(rightShoulder[2]-torso[2])

    rs_vector2[0]=(rightShoulder[0]-rightElbow[0])
    rs_vector2[1]=(rightShoulder[1]-rightElbow[1])
    rs_vector2[2]=(rightShoulder[2]-rightElbow[2])

    norm_rs_vector1=math.sqrt((rs_vector1[0]*rs_vector1[0])+(rs_vector1[1]*rs_vector1[1])+(rs_vector1[2]*rs_vector1[2]))
    norm_rs_vector2=math.sqrt((rs_vector2[0]*rs_vector2[0])+(rs_vector2[1]*rs_vector2[1])+(rs_vector2[2]*rs_vector2[2]))

    rightshoulder_dot_product=(rs_vector1[0]*rs_vector2[0])+(rs_vector1[1]*rs_vector2[1])+(rs_vector1[2]*rs_vector2[2])
    #Now calculating the angle
    try:
        rightshoulder_angle =(math.acos(float(rightshoulder_dot_product) /(float(norm_rs_vector2)*float(norm_rs_vector1))) * 180/pi)
    except ZeroDivisionError:
        pass


    ###################################################################################################### rightelbow_angle calculations
    re_vector1[0]=(rightElbow[0]-rightHand[0])
    re_vector1[1]=(rightElbow[1]-rightHand[1])
    re_vector1[2]=(rightElbow[2]-rightHand[2])

    re_vector2[0]=(rightElbow[0]-rightShoulder[0])
    re_vector2[1]=(rightElbow[1]-rightShoulder[1])
    re_vector2[2]=(rightElbow[2]-rightShoulder[2])

    norm_re_vector1=math.sqrt((re_vector1[0]*re_vector1[0])+(re_vector1[1]*re_vector1[1])+(re_vector1[2]*re_vector1[2]))
    norm_re_vector2=math.sqrt((re_vector2[0]*re_vector2[0])+(re_vector2[1]*re_vector2[1])+(re_vector2[2]*re_vector2[2]))

    rightelbow_dot_product=(re_vector1[0]*re_vector2[0])+(re_vector1[1]*re_vector2[1])+(re_vector1[2]*re_vector2[2])
    #Now calculating the angle
    try:
        rightelbow_angle =(math.acos(float(rightelbow_dot_product) /(float(norm_re_vector2)*float(norm_re_vector1))) * 180/pi)
    except ZeroDivisionError:
        pass
    ###################################################################################################### rightpit_angle calculations
    rp_vector1[0]=(rightShoulder[0]-rightElbow[0])
    rp_vector1[1]=(rightShoulder[1]-rightElbow[1])
    rp_vector1[2]=(rightShoulder[2]-rightElbow[2])

    rp_vector2[0]=(rightShoulder[0]-rightHip[0])
    rp_vector2[1]=(rightShoulder[1]-rightHip[1])
    rp_vector2[2]=(rightShoulder[2]-rightHip[2])

    norm_rp_vector1=math.sqrt((rp_vector1[0]*rp_vector1[0])+(rp_vector1[1]*rp_vector1[1])+(rp_vector1[2]*rp_vector1[2]))
    norm_rp_vector2=math.sqrt((rp_vector2[0]*rp_vector2[0])+(rp_vector2[1]*rp_vector2[1])+(rp_vector2[2]*rp_vector2[2]))

    rightpit_dot_product=(rp_vector1[0]*rp_vector2[0])+(rp_vector1[1]*rp_vector2[1])+(rp_vector1[2]*rp_vector2[2])
    #Now calculating the angle
    try:
        rightpit_angle =(math.acos(float(rightpit_dot_product) /(float(norm_rp_vector2)*float(norm_rp_vector1))) * 180/pi)
    except ZeroDivisionError:
        pass
    ###################################################################################################### lhand_distance calculations
    lhand_distance=math.sqrt((leftHip[0]-leftHand[0])*(leftHip[0]-leftHand[0])+(leftHip[1]-leftHand[1])*(leftHip[1]-leftHand[1]))#+(rightHand[2]-leftHand[2])*(rightHand[2]-leftHand[2]))
    ###################################################################################################### profile_angle calculations
    pa_vector1[0]=rightShoulder[0]-leftShoulder[0]
    pa_vector1[1]=rightShoulder[1]-leftShoulder[1]
    pa_vector1[2]=rightShoulder[2]-leftShoulder[2]

    pa_vector2[0]=torso[0]
    pa_vector2[1]=torso[1]
    pa_vector2[2]=torso[2]

    norm_pa_vector1=math.sqrt((pa_vector1[0]*pa_vector1[0])+(pa_vector1[1]*pa_vector1[1])+(pa_vector1[2]*pa_vector1[2]))
    norm_pa_vector2=math.sqrt((pa_vector2[0]*pa_vector2[0])+(pa_vector2[1]*pa_vector2[1])+(pa_vector2[2]*pa_vector2[2]))

    profile_dot_product=(pa_vector1[0]*pa_vector2[0])+(pa_vector1[1]*pa_vector2[1])+(pa_vector1[2]*pa_vector2[2])
    try:
        profile_angle =(math.acos(float(profile_dot_product) /(float(norm_pa_vector2)*float(norm_pa_vector1))) * 180/pi)
    except ZeroDivisionError:
        pass
    ####################################################################################################### rhandrelativecoordinate calculations
    rhandrelativecoordinate[0]=torso[0]-rightHand[0]
    rhandrelativecoordinate[1]=torso[1]-rightHand[1]
    rhandrelativecoordinate[2]=torso[2]-rightHand[2]

##############################################################################################

import Queue
import threading
import socket

msgQ = Queue.Queue()

class NetworkThread(threading.Thread):
    def __init__(self, host, port):
        super(self.__class__, self).__init__() # initialize the Thread super-class
        self.host = host    # those are going to be used in run function
        self.port = port
        self.s = None

    def run(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s = s
            s.connect((self.host, self.port))
            s.sendall("xname: human_recognition_node\n")
            s.sendall("type: sensor\n")
            s.sendall("action\n")
            while True:
                msg = msgQ.get()
                if msg == "EXIT":
                    break
                s.sendall("trig EVT_HMNR9N " + str(len(msg)) + "\n")
                s.sendall(msg)
                s.sendall("\n")
            s.close()
        except socket.error as msg:
            s.close()

if __name__== "__main__":
    thread = NetworkThread("144.122.206.93", 8000)
    thread.start()
    try:
        initialise()
    except rospy.ROSInterruptException:
            pass

    msgQ.put("EXIT")
    if not (thread.s is None):
        try:
            thread.s.close()
        except:
            pass
    thread.join()
