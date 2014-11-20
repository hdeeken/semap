#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from sqlalchemy.sql import func
from geoalchemy2.elements import (
    WKTElement, WKBElement, RasterElement, CompositeElement)
    
from geoalchemy2.functions import ST_Distance, ST_AsText

from db_environment import Session
from db_model import Object, Pose

from geometry_msgs.msg import PoseStamped as ROSPose

from spatial_db.srv import *
from spatial_db.msg import Object as ROSObject

import rospy

def add_object(req):
    session = Session()
    for rosobject in req.objects:
      object = Object()
      object.fromROS(rosobject)
      session.add(object)
    session.commit()
    return

def get_all_objects(req):
    res = GetObjectsResponse()
    session = Session()
    for obj in session.query(Object):
      res.objects.append(obj.toROS())
    return

def spatial_db_services():
    rospy.init_node('spatial_db_services')
    srv_add_object = rospy.Service('add_object', AddObject, add_object)
    print "SpatialDB Services are online."
    rospy.spin()

if __name__ == "__main__":
    spatial_db_services()
