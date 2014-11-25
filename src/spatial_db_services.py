#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from sqlalchemy.sql import func
from sqlalchemy.orm import aliased

from geoalchemy2 import Geometry
#from geoalchemy2.functions import GenericFunction
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from postgis_functions import *

from db_environment import Session
from db_model import *

from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from spatial_db.msg import PolygonMesh as ROSPolygonMesh
from spatial_db.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db.msg import ObjectDescription as ROSObjectDescription
from spatial_db.msg import ObjectInstance as ROSObjectInstance
from spatial_db.msg import ObjectInstanceOverview as ROSObjectInstanceOverview

from spatial_db.srv import *

from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix

import rospy

def add_object_descriptions(req):
    session = Session()
    for desc in req.descriptions:
      print desc.type, 'will be added'
      object = ObjectDescription()
      object.fromROS(desc)
      session.add(object)
    session.commit()
    return

def add_object_instances(req):
    session = Session()
    for obj in req.objects:
      object = ObjectInstance()
      object.fromROS(obj)
      session.add(object)
    session.commit()
    return

def get_all_object_instances(req):
    res = GetAllObjectInstancesResponse()
    session = Session()
    for obj in session.query(ObjectInstance):
      res.objects.append(obj.toROS())
    return res

def truncate_all_tables(req):
    truncate_all()
    return

def create_database(req):
    create_all()
    return

def drop_database(req):
    drop_all()
    return

def spatial_db_services():
    rospy.init_node('spatial_db_services')

#
#object descriptions
# total num
# list of types
# list of model_types per type


    ## Object Description
    #srv_object_description_overview
    #srv_all_object_description
    #srv_get_all_object_description
    #srv_remove_object_description
    srv_add_object_descriptions = rospy.Service('add_object_descriptions', AddObjectDescriptions, add_object_descriptions)

    ## Object Instances
    srv_add_object_instances = rospy.Service('add_object_instances', AddObjectInstances, add_object_instances)
    srv_get_all_object_instances = rospy.Service('get_all_object_instances', GetAllObjectInstances, get_all_object_instances)

    srv_truncate_all_tables = rospy.Service('truncate_all_tables', TruncateAllTables, truncate_all_tables)
    srv_create_database = rospy.Service('create_database', CreateDatabase, create_database)
    srv_drop_database = rospy.Service('drop_database', DropDatabase, drop_database)

    print "SpatialDB Services are online."
    rospy.spin()

if __name__ == "__main__":
    #truncate_all()
    spatial_db_services()
