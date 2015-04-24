#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

import rospy
from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sets import Set
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

from db_pose_model import *
from ros_postgis_conversion import *

### GEOMETRY TABLES

""" GeometryModel
# holds the various possible 3d geometries that can be identified for an object
# type: defines the purpose of this geometry (e.g. 3d surface)
# geometry_type: defines the underlying geometry (e.g. 3d triangle mesh)
# geometry: the geometry itself as binary blob
"""

class GeometryModel(Base):
  __tablename__ = 'geometry_model'
  id = Column('id', Integer, primary_key=True)
  type = Column('type ', String)
  object_description_id = Column('object_decription_id', Integer, ForeignKey('object_description.id'), nullable=False)
  object_description = relationship("ObjectDescription", backref=backref('geometry_models', order_by=type))
  pose_id = Column('pose_id', Integer, ForeignKey('local_pose.id'), nullable=True)
  pose = relationship("LocalPose", backref=backref('geometry_model', uselist=False))
  geometry_type = Column('geometry_type ', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRYZ', dimension=3), nullable=False)

  # 2D Geometry

  def transformed(self):
    return self.pose.apply(self.geometry)

  def fromROSPoint2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.pose = fromROSPose(nullPose())
    self.geometry_type = 'POINT2D'
    self.geometry =  fromPoint2D(model.geometry)

  def toROSPoint2DModel(self):
    ros = Point2DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.geometry = toPoint2D(self.geometry)
    return ros

  def fromROSPose2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    matrix = euler_matrix(0.0, 0.0, model.pose.theta)
    matrix[0][3] = model.pose.x
    matrix[1][3] = model.pose.y
    self.pose.pose = fromMatrix(matrix)
    self.geometry_type = 'POSE2D'
    geometry_string = 'POINT(%f %f %f)' % (model.pose.x, model.pose.y, 0.0)
    self.geometry =  WKTElement(geometry_string)

  def toROSPose2DModel(self):
    ros = Pose2DModel()
    ros.id = self.id
    ros.type = str(self.type)
    matrix = toMatrix(self.pose.pose)
    ros.pose.x = matrix[0][3]
    ros.pose.y = matrix[1][3]
    ros.pose.theta = euler_from_matrix(matrix)[2]
    return ros

  def fromROSPolygon2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGON2D'
    self.geometry = fromPolygon2D(model.geometry)

  def toROSPolygon2DModel(self):
    ros = Polygon2DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    ros.geometry = toPolygon2D(self.geometry)
    return ros

  # 3D Geometry

  def fromROSPoint3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(nullPose())
    self.geometry_type = 'POINT3D'
    self.geometry =  fromPoint3D(model.geometry)

  def toROSPoint3DModel(self):
    ros = Point3DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.geometry = toPoint3D(self.geometry)
    return ros

  def fromROSPose3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POSE3D'
    self.geometry = fromPoint3D(model.pose.position)

  def toROSPose3DModel(self):
    ros = Pose3DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    return ros

  def fromROSPolygon3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGON3D'
    self.geometry = fromPolygon3D(model.geometry)

  def toROSPolygon3DModel(self):
    ros = Polygon3DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    ros.geometry = toPolygon3D(self.geometry)
    return ros

  def fromROSTriangleMesh3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'TRIANGLEMESH3D'
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry = fromTriangleMesh3D(model.geometry)

  def toROSTriangleMesh3DModel(self):
    ros = TriangleMesh3DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    ros.geometry = toTriangleMesh3D(self.geometry)
    return ros

  def fromROSPolygonMesh3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGONMESH3D'
    self.geometry = fromPolygonMesh3D(model.geometry)

  def toROSPolygonMesh3DModel(self):
    ros = PolygonMesh3DModel()
    ros.id = self.id
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    ros.pose = self.pose.toROS()
    ros.geometry = toPolygonMesh3D(self.geometry)
    return ros
