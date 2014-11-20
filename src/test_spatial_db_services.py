#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from sqlalchemy.sql import func
from geoalchemy2.elements import (
    WKTElement, WKBElement, RasterElement, CompositeElement)
    
from geoalchemy2.functions import ST_Distance, ST_AsText

from db_environment import Session
from db_model import Object, Pose

from geometry_msgs.msg import PoseStamped

from spatial_db.srv import *
from spatial_db.msg import *
import rospy

def test_add_object():
	try:
		rospy.wait_for_service('add_object')
		request = AddObjectRequest()

		testObject1 = Object()
		testObject1.name = 'Test1'
		testObject1.type = 'Type1'
		testObject1.pose.header.frame_id = 'testFrame1'
		testObject1.pose.pose.position.x = 2.0
		testObject1.pose.pose.position.y = 0.0
		testObject1.pose.pose.position.z = 0.0
		testObject1.pose.pose.orientation.x = 0.0
		testObject1.pose.pose.orientation.y = 0.0
		testObject1.pose.pose.orientation.z = 0.0
		testObject1.pose.pose.orientation.w = 1.0
		request.objects.append(testObject1)

		testObject2 = Object()
		testObject2.name = 'Test2'
		testObject2.type = 'Type2'
		testObject2.pose.header.frame_id = 'testFrame2'
		testObject2.pose.pose.position.x = 2.0
		testObject2.pose.pose.position.y = 2.0
		testObject2.pose.pose.position.z = 0.0
		testObject2.pose.pose.orientation.x = 0.0
		testObject2.pose.pose.orientation.y = 0.0
		testObject2.pose.pose.orientation.z = 0.0
		testObject2.pose.pose.orientation.w = 1.0
		request.objects.append(testObject2)

		add_object_call = rospy.ServiceProxy('add_object', AddObject)
		response = add_object_call(request)
		print 'Add Object service call succeeded!'
		return True
	except rospy.ServiceException as e:
		return False, "AddObject service call failed: %s" % e
    
if __name__ == "__main__":
    test_add_object()
