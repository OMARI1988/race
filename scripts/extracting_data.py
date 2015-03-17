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

# manage diroctories to store data and images
image_dir = '/home/omari/race/LAD/images/'
person = 'breakfast_aryana_1'

if not os.path.isdir(image_dir):
	print 'please change the diroctory in extract_data.py'

if not os.path.isdir(image_dir+person+'/'):
	os.makedirs(image_dir+person)
	os.makedirs(image_dir+person+'/'+'original')
	os.makedirs(image_dir+person+'/'+'processed')
if os.path.isfile(image_dir+person+'/data.txt'):
	os.remove(image_dir+person+'/data.txt')

fx = 525.0									# xtion calibration
fy = 525.0									# xtion calibration
cx = 319.5									# xtion calibration
cy = 239.5									# xtion calibration

# edit the next line if you want to add all the skeleton parts
frames_of_skeleton = ['/head_', '/neck_', '/right_shoulder_', '/right_elbow_', '/right_hand_', '/left_shoulder_', '/left_elbow_', '/left_hand_']
number_of_skeletons = 8								# maximum number of skeletons to be tracked (openni_tracker = 8)
number_of_objects = 30								# maximum number of objects to be tracked

if __name__ == '__main__':

    img = []
    obj = {}
    idd = {}									# keep track of object ids to remove the non continous ones
    br = CvBridge()								# Create a black image, a window

    def detect_and_draw(imgmsg):						# call back for image topic
	global img
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

    def object_callback(data):							# call back for image topic
	global obj
	for i in data.markers:
		if i.id not in obj:
			obj[i.id] = {}
		obj[i.id][i.ns] = i
		idd[i.id] = 0

	for j in obj.keys():
		idd[j]+=1
		if idd[j] == 15:
			idd.pop(j, None)
			obj.pop(j, None)



    rospy.init_node('extract_info')						# initialize node

    image_topic = rospy.resolve_name("/head_mount_kinect/rgb/image_color") 	# image topic
    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)	# subscribe to image topic

    object_topic = rospy.resolve_name("/perception/tracking/neat_markers") 	# object topic
    rospy.Subscriber(object_topic, MarkerArray, object_callback)			# subscribe to object topic

    listener = tf.TransformListener()						# listining to all tf frames

    rate = rospy.Rate(15.0)							# define the rate
    frame_number = 1								# frame counter

    # Main loop
    while not rospy.is_shutdown():
	
	msg = []								# this msg will be written in the file of data.txt
	msg.append(str(frame_number)+'\n')
	
	skeleton_pose = {}							# skeleton pose data
	skeleton_ori = {}							# skeleton orientation data
	objects_pose = {}							# object data
	objects_ori = {}							# object data

	# read all skeletons up to 8
	for i in range(number_of_skeletons):
	    skeleton_pose[i] = []
   	    skeleton_ori[i] = []
	    for j in frames_of_skeleton:
		try:
		    (trans,rot) = listener.lookupTransform('/head_mount_kinect_link', j+str(i), rospy.Time(0))
		    #print j+str(i),int(trans[0]*10000)/10000.0,int(trans[1]*10000)/10000.0,int(trans[2]*10000)/10000.0
		    skeleton_pose[i].append(trans)
		    skeleton_ori[i].append(rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    1

	# read all objects up to 30
	for i in range(number_of_objects):
            try:
		(trans,rot) = listener.lookupTransform('/head_mount_kinect_rgb_frame', '/perception/pipeline'+str(i)+'/tracker', rospy.Time(0))
		#print 'obj_'+str(i),int(trans[0]*10000)/10000.0,int(trans[1]*10000)/10000.0,int(trans[2]*10000)/10000.0
		#print 'obj_'+str(i),rot
		objects_pose[i] = trans
		objects_ori[i] = rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		1

	obj_cp = obj.copy()	# copy the most recent object topic
	

	# append the msg for the data.txt file with skeleton data
	for i in skeleton_pose:
		if skeleton_pose[i] != []:
			msg.append('skeleton_'+str(i)+':')
			msg.append('\n')
			msg.append('pose:')
			msg.append('\n')
			for j in range(len(skeleton_pose[i])):
				msg.append(frames_of_skeleton[j][1:-1]+','+str(skeleton_pose[i][j][0])+','+str(skeleton_pose[i][j][1])+','+str(skeleton_pose[i][j][2]))
				msg.append('\n')
			msg.append('orientation:')
			msg.append('\n')
			for j in range(len(skeleton_ori[i])):
				var = skeleton_ori[i][j]
				msg.append(frames_of_skeleton[j][1:-1]+','+str(var[0])+','+str(var[1])+','+str(var[2])+','+str(var[3]))
				msg.append('\n')

	"""
	# append the msg for the data.txt file with objects data
	for i in objects_pose:
		if objects_pose[i] != []:
			msg.append('object_'+str(i)+':')
			msg.append('\n')
			msg.append('pose:')
			msg.append('\n')
			for j in range(len(objects_pose[i])):
				msg.append(str(objects_pose[i][j][0])+','+str(objects_pose[i][j][1])+','+str(objects_pose[i][j][2]))
				msg.append('\n')
			msg.append('orientation:')
			msg.append('\n')
			for j in range(len(objects_ori[i])):
				var = objects_ori[i][j]
				msg.append(str(var[0])+','+str(var[1])+','+str(var[2])+','+str(var[3]))
				msg.append('\n')
	"""



	# process image (plot skeleton and objects)
        if img != []:
	    img_original = img.copy()
	    img_processed = img.copy()
	    skeleton_flag = 0									# to make sure a full skeleton is detected

	    # scan through all possible number of skeletons
	    for sk_num in range(number_of_skeletons):

	    	if len(skeleton_pose[sk_num])==len(frames_of_skeleton):
			skeleton_flag = 1							# a full skeleton was detected
			x = []
			y = []
			# plotting skeletons
			for i in range(len(skeleton_pose[sk_num])):
				x.append(int(-skeleton_pose[sk_num][i][1] * fx / skeleton_pose[sk_num][i][0] + cx))
				y.append(int(-skeleton_pose[sk_num][i][2] * fy / skeleton_pose[sk_num][i][0] + cy))

			if x != []:
				cv2.line(img_processed,(x[0],y[0]),(x[1],y[1]),(0,0,0),2)	#head to neck
				cv2.line(img_processed,(x[1],y[1]),(x[2],y[2]),(0,0,0),2)	#neck to r_shoulder
				cv2.line(img_processed,(x[1],y[1]),(x[5],y[5]),(0,0,0),2)	#neck to l_shoulder
				cv2.line(img_processed,(x[2],y[2]),(x[3],y[3]),(0,0,0),2)	#r_shoulder to r_elbow
				cv2.line(img_processed,(x[5],y[5]),(x[6],y[6]),(0,0,0),2)	#l_shoulder to l_elbow
				cv2.line(img_processed,(x[4],y[4]),(x[3],y[3]),(0,0,0),2)	#r_elbow to r_hand
				cv2.line(img_processed,(x[6],y[6]),(x[7],y[7]),(0,0,0),2)	#l_elbow to l_hand

			for i in range(len(skeleton_pose[sk_num])):
				if i == 0:
					cv2.circle(img_processed,(x[i],y[i]),5,(0,255,0),-1)
				else:
					cv2.circle(img_processed,(x[i],y[i]),5,(0,0,255),-1)

	    # plotting all objects
	    if skeleton_flag == 1 and objects_pose != {}:
			T = {}
			R = {}

			for i in objects_pose:
				x, y, z = objects_pose[i][0], objects_pose[i][1], objects_pose[i][2]
				T[i] = translation_matrix((x, y, z))
				R[i] = quaternion_matrix(objects_ori[i])

			obj_data = {}
			for i in obj_cp:
			    if i not in R:							# sometimes R has a missing element due to delay
				    print 'R has a missing value !'
			    else:
				    obj_data[i] = {}
				    obj_data[i]['x_axis1'] = []
				    obj_data[i]['x_axis2'] = []
				    obj_data[i]['y_axis1'] = []
				    obj_data[i]['y_axis2'] = []
				    obj_data[i]['z_axis1'] = []
				    obj_data[i]['z_axis2'] = []
				    obj_data[i]['wire1'] = []
				    obj_data[i]['wire2'] = []

				    for j in obj_cp[i]:
					    for k in range(0,len(obj_cp[i][j].points),2):
						p1 = obj_cp[i][j].points[k]
						p2 = obj_cp[i][j].points[k+1]
						P1 = numpy.dot(R[i], translation_matrix((p1.x,p1.y,p1.z)))
						P2 = numpy.dot(R[i], translation_matrix((p2.x,p2.y,p2.z)))
						P1 = numpy.dot(T[i], P1)
						P2 = numpy.dot(T[i], P2)

					    	if j == 'axes_x':
							obj_data[i]['x_axis1'] = [P1[0][3],P1[1][3],P1[2][3]]
							obj_data[i]['x_axis2'] = [P2[0][3],P2[1][3],P2[2][3]]
					    	if j == 'axes_y':
							obj_data[i]['y_axis1'] = [P1[0][3],P1[1][3],P1[2][3]]
							obj_data[i]['y_axis2'] = [P2[0][3],P2[1][3],P2[2][3]]
					   	if j == 'axes_z':
							obj_data[i]['z_axis1'] = [P1[0][3],P1[1][3],P1[2][3]]
							obj_data[i]['z_axis2'] = [P2[0][3],P2[1][3],P2[2][3]]
					    	if j == 'wireframe':
							obj_data[i]['wire1'].append([P1[0][3],P1[1][3],P1[2][3]])
							obj_data[i]['wire2'].append([P2[0][3],P2[1][3],P2[2][3]])
					
			# plotting objects
			for i in obj_data:
			    for j in ['x','y','z']:
				x1_x = int(-obj_data[i][j+'_axis1'][1] * fx / obj_data[i][j+'_axis1'][0] + cx)
				x1_y = int(-obj_data[i][j+'_axis1'][2] * fy / obj_data[i][j+'_axis1'][0] + cy)
				x2_x = int(-obj_data[i][j+'_axis2'][1] * fx / obj_data[i][j+'_axis2'][0] + cx)
				x2_y = int(-obj_data[i][j+'_axis2'][2] * fy / obj_data[i][j+'_axis2'][0] + cy)

				#x_axis
				if j == 'x':
					cv2.line(img_processed,(x1_x,x1_y),(x2_x,x2_y),(0,0,255),2)	#x
				#y_axis
				if j == 'y':
					cv2.line(img_processed,(x1_x,x1_y),(x2_x,x2_y),(0,255,0),2)	#y
				#z_axis
				if j == 'z':
					cv2.line(img_processed,(x1_x,x1_y),(x2_x,x2_y),(255,0,0),2)	#z

			    for k in range(len(obj_data[i]['wire1'])):				# the wireframe around object
				x1_x = int(-obj_data[i]['wire1'][k][1] * fx / obj_data[i]['wire1'][k][0] + cx)
				x1_y = int(-obj_data[i]['wire1'][k][2] * fy / obj_data[i]['wire1'][k][0] + cy)
				x2_x = int(-obj_data[i]['wire2'][k][1] * fx / obj_data[i]['wire2'][k][0] + cx)
				x2_y = int(-obj_data[i]['wire2'][k][2] * fy / obj_data[i]['wire2'][k][0] + cy)
				cv2.line(img_processed,(x1_x,x1_y),(x2_x,x2_y),(200,200,200),2)	

	    		cv2.imshow('rgb',img_original)
	    		cv2.imshow('processed',img_processed)
			k = cv2.waitKey(1) & 0xFF

			print 'frame_number = ',frame_number
			print '---------------------------------'
			with open(image_dir+person+'/data.txt', 'a') as file:
			    for j in msg:
			    	file.write(str(j))
			frame_number+=1
			cv2.imwrite(image_dir+person+'/'+'original'+'/'+str(frame_number)+'.png', img_original) 
			cv2.imwrite(image_dir+person+'/'+'processed'+'/'+str(frame_number)+'.png', img_processed) 
			rate.sleep()

