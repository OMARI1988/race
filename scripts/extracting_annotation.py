#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
from visualization_msgs.msg import MarkerArray
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from transformations import *
import os
import shutil

# manage diroctories to store data and images
image_dir = '/home/omari/race/LAD/images/'
subactivity_dir = '/home/omari/race/LAD/subactivities/'
annotation_dir = '/home/omari/race/LAD/annotations/muhannad/'
all_person = ['breakfast_aryana_1','breakfast_aryana_2','breakfast_eris_1','breakfast_eris_2','breakfast_eris_3','breakfast_eris_4','breakfast_paul_1','breakfast_paul_2','breakfast_paul_3','breakfast_paul_4']
all_action = ['pickup','putdown']

if not os.path.isdir(image_dir):
	print 'please change the image diroctory in extract_data.py'

if not os.path.isdir(subactivity_dir):
	print 'please change the subactivity diroctory in extract_data.py'

if not os.path.isdir(annotation_dir):
	print 'please change the annotation diroctory in extract_data.py'



if __name__ == '__main__':

	for person in all_person:
	    for action in all_action:

		if not os.path.isdir(subactivity_dir+action+'/'):
			os.makedirs(subactivity_dir+action)

		for i in os.walk(subactivity_dir+action+'/'):
			folders = i[1]
			break

		new_folder = len(folders)+1

		frame1 = []
		frame2 = []
		data = open(annotation_dir+person+'.txt', 'r')
		for line in data.read().split('\n'):
			if line.split(',')[0] == action:
				frame1.append(int(line.split(',')[1]))
				frame2.append(int(line.split(',')[2]))

		print frame1,frame2
	
		#frame1 = 3
		#frame2 = 3
		printing = 0
		for i in range(len(frame1)):
			os.makedirs(subactivity_dir+action+'/'+str(new_folder))
			os.makedirs(subactivity_dir+action+'/'+str(new_folder)+'/original')
			os.makedirs(subactivity_dir+action+'/'+str(new_folder)+'/processed')
			new_data = open(subactivity_dir+action+'/'+str(new_folder)+'/'+'data.txt', 'w')
			data = open(image_dir+person+'/'+'data.txt', 'r')
			print i,frame1[i]
			print '---------------------'
			for line in data.read().split('\n'):
				if line.split(',')[0] == str(frame1[i]):
					printing = 1
				if line.split(',')[0] == str(frame2[i]+1):
					printing = 0
				if printing == 1:
					new_data.write(line+'\n')

			for j in range(frame1[i],frame2[i]):
				shutil.copyfile(image_dir+person+'/original/'+str(j)+'.png', subactivity_dir+action+'/'+str(new_folder)+'/original/'+str(j)+'.png')
				shutil.copyfile(image_dir+person+'/processed/'+str(j)+'.png', subactivity_dir+action+'/'+str(new_folder)+'/processed/'+str(j)+'.png')

			new_data.close()
			new_folder+=1














			
			
