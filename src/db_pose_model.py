#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')
from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sets import Set
from db_environment import Base
from db_environment import Session
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from spatial_db_msgs.msg import PolygonMesh as ROSPolygonMesh
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription
from spatial_db_msgs.msg import ObjectInstance as ROSObjectInstance

from numpy import radians
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix

#from db_geometry_model import *
#from db_object_model import *

### POSE TABLES

#geometry ST_Affine(geometry geomA, float a, float b, float c,
#                                   float d, float e, float f,
#                                   float g, float h, float i,
#                                   float xoff, float yoff, float zoff)

class LocalPose(Base):
  __tablename__ = 'local_pose'
  id = Column('id', Integer, primary_key=True)
  pose = Column('pose', String)

  def toROS(self):
     return toROSPose(self.pose)

  def fromROS(self, ros):
    self.pose = fromROSPose(ros)
    return

  def apply(self, geometry):
    matrix = toMatrix( self.pose)
    session = Session()
    transformed_geometry = session.execute(ST_Affine(geometry, matrix[0][0], matrix[0][1], matrix[0][2], \
                                               matrix[1][0], matrix[1][1], matrix[1][2], \
                                               matrix[2][0], matrix[2][1], matrix[2][2], \
                                               matrix[0][3], matrix[1][3], matrix[2][3])).scalar()
    return transformed_geometry

class GlobalPose(Base):
  __tablename__ = 'global_pose'
  id = Column('id', Integer, primary_key=True)
  ref_system = Column('ref_system', String)
  pose = Column('pose', String)

  def toROS(self):
    ros = ROSPoseStamped()
    ros.header.frame_id = str(self.ref_system)
    ros.pose = toROSPose(self.pose)
    return ros

  def fromROS(self, ros):
    self.ref_system = ros.header.frame_id
    self.pose = fromROSPose(ros.pose)

  def apply(self, geometry):
    matrix = self.pose.toMatrix()
    session = Session()
    transformed_geometry = session.execute(ST_Affine(geometry, matrix[0][0], matrix[0][1], matrix[0][2], \
                                               matrix[1][0], matrix[1][1], matrix[1][2], \
                                               matrix[2][0], matrix[2][1], matrix[2][2], \
                                               matrix[0][3], matrix[1][3], matrix[2][3])).scalar()
    return transformed_geometry

### POSE FUNCTIONS

'''
creates a transformation in form a 4x4 matrix
first 3 values give the x,y, offset
follwoing 9 values give the 3x3 rotation matrix
the bottom line 0, 0, 0, 1 must be set after reading
'''

def fromMatrix(matrix):
  string = '%f %f %f %f, %f %f %f %f, %f %f %f %f' \
  % (matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3], \
     matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3], \
     matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3])
  return string

def toMatrix(string):
  rows_ = string.split(',')
  abc_xoff = [float(x) for x in rows_[0].split()]
  def_yoff = [float(x) for x in rows_[1].split()]
  ghi_zoff = [float(x) for x in rows_[2].split()]
  matrix = [abc_xoff, def_yoff, ghi_zoff,[0,0,0,1]]
  return matrix

def fromROSPose(ros_pose):
  quaternion = [ros_pose.orientation.x, ros_pose.orientation.y, \
          ros_pose.orientation.z, ros_pose.orientation.w]
  matrix = quaternion_matrix(quaternion)
  matrix[0][3] = ros_pose.position.x
  matrix[1][3] = ros_pose.position.y
  matrix[2][3] = ros_pose.position.z
  return fromMatrix(matrix)

def toROSPose(db_pose):
  ros_pose = ROSPose()
  matrix = toMatrix(db_pose)
  quaternion = quaternion_from_matrix(matrix)
  ros_pose.position.x = matrix[0][3]
  ros_pose.position.y = matrix[1][3]
  ros_pose.position.z = matrix[2][3]
  ros_pose.orientation.x = quaternion[0]
  ros_pose.orientation.y = quaternion[1]
  ros_pose.orientation.z = quaternion[2]
  ros_pose.orientation.w = quaternion[3]
  return ros_pose

def nullPose():
  ros_pose = ROSPose()
  ros_pose.position.x = 0.0
  ros_pose.position.y = 0.0
  ros_pose.position.z = 0.0
  ros_pose.orientation.x = 0.0
  ros_pose.orientation.y = 0.0
  ros_pose.orientation.z = 0.0
  ros_pose.orientation.w = 1.0
  return ros_pose
