#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 19:38:45 2021

@author: joseadr
"""

import rospy
import rospkg
import rosbag
import yaml

import os.path
import time
import numpy as np
from os.path import join

#include "std_msgs/String.h"
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#from std_msgs.msg import String

from dvs_msgs.msg import Event, EventArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo



def make_camera_msg(cam):
    camera_info_msg = CameraInfo()
    width, height = cam[0], cam[1]
    fx, fy = cam[2], cam[3]
    cx, cy = cam[4], cam[5]
    camera_info_msg.distortion_model="plumb_bob"
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.K = [fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1]
                         
    camera_info_msg.D = [0, 0, 0, 0]
    
    camera_info_msg.P = [fx, 0, cx, 0,
                         0, fy, cy, 0,
                         0, 0, 1, 0]
    return camera_info_msg
    

    
def make_pose_msg(position, orientation, timestamp):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = timestamp
    pose_msg.header.frame_id = '/cam'
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    pose_msg.pose.orientation.x = orientation[0]
    pose_msg.pose.orientation.y = orientation[1]
    pose_msg.pose.orientation.z = orientation[2]
    pose_msg.pose.orientation.w = orientation[3]
    return pose_msg    


""" Log with a small offset to avoid problems at zero"""
def safe_log(img):
    eps = 0.001
    return np.log(eps + img)
    

#https://github.com/uzh-rpg/rpg_davis_simulator/blob/50e84063fffb9de609091df2f77b2ad52c3008fa/src/dvs_simulator_py/dataset_utils.py#L36
""" Parse a dataset folder """
def parse_dataset(cam_dir,dataset_dir):
    
     # Parse camera calibration
    cam_file = open('%s/camera_prueba.yaml' % cam_dir)
    cam_data = yaml.safe_load(cam_file)

    image_data = {}
    tiempos=[]
    posiciones=[]
    orientaciones=[]
    with open('%s/svo_prueba_traslacion1.txt' % dataset_dir) as fichero:
       linea = fichero.readline() 
       contador = 1
       while linea:
          if contador==4:
             secs = int(linea.split(': ')[1])
             #print(secs)
          elif contador == 5:
             nsecs = int(linea.split(': ')[1])
             #print(nsecs,"\n")
             tiempo = rospy.Time(secs,nsecs)
             tiempos.append(tiempo)
          elif contador == 10:
             posicion=[]
             posicion.append(float(linea.split(': ')[1]))
          elif contador == 11:
             posicion.append(float(linea.split(': ')[1]))
          elif contador == 12:
             posicion.append(float(linea.split(': ')[1]))
             posiciones.append(posicion)
          elif contador == 14:
             orientacion=[]
             orientacion.append(float(linea.split(': ')[1]))
          elif contador == 15:
             orientacion.append(float(linea.split(': ')[1]))
          elif contador == 16:
             orientacion.append(float(linea.split(': ')[1]))
          elif contador == 17:
             orientacion.append(float(linea.split(': ')[1]))
             orientaciones.append(orientacion)
          elif contador == 19:
             contador = 0
          linea = fichero.readline()
          contador += 1
    #print("Tamano de posiciones= ",len(posiciones))
    
    ###############################
    ###VIENE DE CAMERA_ATAN.YAML###
    ###############################
    
    width = cam_data['cam_width']
    height = cam_data['cam_height']
    fx = cam_data['cam_fx']
    fy = cam_data['cam_fy']
    cx = cam_data['cam_cx']
    cy = cam_data['cam_cy']
    
    cam = [width, height, fx, fy, cx, cy]
    
    return tiempos, posiciones, orientaciones, cam

#t,pos,ori,cam = parse_dataset('.','.')

#print(t[0], "t1= ",t[1], " pos = ", pos[0][0], ", ",pos[0][1], ", ",pos[0][2])

def make_event(x, y, ts, pol):
    e = Event()
    e.x = x
    e.y = y
    e.ts = rospy.Time(secs=ts)
    e.polarity = pol
    return e

def parse_events(dataset_dir,timestamp,last_pub,limite):
    eventos = []
    with open('%s/v2e-dvs-events.txt' % dataset_dir) as fichero:
        linea = fichero.readline() 
        contador = 1
        print("Empieza a leer eventos")
        while linea:
          #print(contador)
          if contador<limite:  
             contador += 1
             if(contador==limite-1):
                print("Se ha saltado la cabecera = ", limite)
          else:
             #print("entrooooo")
             time = float(linea.split(' ')[0])
             #if (contador%10000==1):
                #print("Aqui es = ",rospy.Time.from_sec(time), "con timestamp ",timestamp, " y last pub = ",last_pub, " hasta ", timestamp)
             if( rospy.Time.from_sec(time)<timestamp and rospy.Time.from_sec(time) > last_pub ):
                #print("Aqui es = ",rospy.Time.from_sec(time), "con timestamp ",timestamp, " y ",last_pub)
                x = int(linea.split(' ')[1])
                y = int(linea.split(' ')[2])
                polarity = bool(linea.split(' ')[2])
                eventos.append(make_event(x, y, time, polarity))
                contador += 1
                if contador%10000==1:
                   
                   print("contador = ",contador)
             elif(rospy.Time.from_sec(time) > timestamp):
                print("Tamano de eventos es",len(eventos))
                limite = contador
                break
             else:
                contador += 1
          
          linea = fichero.readline()
          #print(linea.split(' ')[0])
          
             
    return eventos, limite
        
        
        
if __name__ == '__main__':
    """
    package_dir = rospack.get_path('dvs_simulator_py')

    # Load simulator parameters
    dataset_name = rospy.get_param('dataset_name', '')
    # Parse dataset
    dataset_dir = os.path.join(package_dir, 'datasets', 'full_datasets', dataset_name, 'data')
    """
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path('crear_rosbag')
    
    times,positions,orientations,cam = parse_dataset(package_dir,package_dir)
    
    ns = rospy.get_param('ns', '/dvs')
    
    camera_info_msg = make_camera_msg(cam)
    #depthmap_topic = '{}/depthmap'.format(ns)
    #image_topic = '{}/image_raw'.format(ns)
    pose_topic = '/optitrack/davis'
    camera_info_topic = '{}/camera_info'.format(ns)
    event_topic = '{}/events'.format(ns)
    
    # Prepare publishers
    #bridge = CvBridge()
    #pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=0)
    #camera_info_pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=0)
    #event_pub = rospy.Publisher(event_topic, EventArray, queue_size=0)
    
    init_time = times[0]
    
    # Debe estar en el mismo directorio
    bag = rosbag.Bag(join(package_dir, '{}-{}.bag'.format("prueba_traslacion1", time.strftime("%Y%m%d-%H%M%S"))), 'w')
    
    bag.write(topic=pose_topic, msg=make_pose_msg(positions[0], orientations[0], init_time), t=init_time)
    
    last_pub_event_timestamp = init_time
    events = []
    limite = 7
    delta_event = rospy.Duration(1.0 / 300)
    for frame_id in range(1, len(times)):
        print("Va por=", frame_id, "(", times[frame_id],") de ",len(times))
        timestamp = times[frame_id]
        
        bag.write(topic=pose_topic, msg=make_pose_msg(positions[frame_id], orientations[frame_id], timestamp), t=timestamp)
        
        bag.write(topic=camera_info_topic, msg=camera_info_msg, t=timestamp)
        
        print("llego aqui1")
    # # compute events for this frame
    #     img = dataset_utils.safe_log(img)
    #     current_events = sim.update(timestamp.to_sec(), img)
    #     events += current_events
    
        events, limite = parse_events(package_dir,timestamp,last_pub_event_timestamp,limite)
        print("llego aqui2")
    # publish events
        
        if timestamp - last_pub_event_timestamp > delta_event:
            events = sorted(events, key=lambda e: e.ts)
            event_array = EventArray()
            event_array.header.stamp = timestamp
            event_array.width = cam[0]
            event_array.height = cam[1]
            event_array.events = events
            #event_pub.publish(event_array)

            bag.write(topic=event_topic, msg=event_array, t=timestamp)
            
            events = []
            last_pub_event_timestamp = timestamp
    bag.close()
    rospy.loginfo('Finished writing rosbag')
    
"""
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
"""
