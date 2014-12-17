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
from db_environment import Base, db
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
from db_transformation_tree_model import *
from db_geometry_model import *

### OBJECT TABLES

""" ObjectDescription
# defines the geometric appearence of a object of a specific type, and is the blueprint object instances
# type: defines the type of object (e.g. Table)
# geometry_model2d: specifies all 2d geometric models bound to this object
# geometry_model3d: specifies all 2d geometric models bound to this object
# FUTURE: default model specifies which models are drawn on default
"""

class ObjectDescription(Base):
  __tablename__ = 'object_description'
  id = Column('id', Integer, primary_key=True)
  type = Column('type', String)
  object_instances = relationship("ObjectInstance", backref="object_description")

  def fromROS(self, ros):
    self.type = ros.type
    for model in ros.point2d_models:
        new = GeometryModel()
        new.fromROSPoint2DModel(model)
        self.geometry_models.append(new)
    for model in ros.pose2d_models:
        new = GeometryModel()
        new.fromROSPose2DModel(model)
        self.geometry_models.append(new)
    for model in ros.polygon2d_models:
        new = GeometryModel()
        new.fromROSPolygon2DModel(model)
        self.geometry_models.append(new)
    for model in ros.point3d_models:
        new = GeometryModel()
        new.fromROSPoint3DModel(model)
        self.geometry_models.append(new)
    for model in ros.pose3d_models:
        new = GeometryModel()
        new.fromROSPose3DModel(model)
        self.geometry_models.append(new)
    for model in ros.polygon3d_models:
        new = GeometryModel()
        new.fromROSPolygon3DModel(model)
        self.geometry_models.append(new)
    for model in ros.trianglemesh3d_models:
        new = GeometryModel()
        new.fromROSTriangleMesh3DModel(model)
        self.geometry_models.append(new)
    for model in ros.polygonmesh3d_models:
        new = GeometryModel()
        new.fromROSPolygonMesh3DModel(model)
        self.geometry_models.append(new)
    return

  def toROS(self):
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = str(self.type)
    for model in self.geometry_models:
      if model.geometry_type == 'POINT2D':
        ros.point2d_models.append(model.toROSPoint2DModel())
      elif model.geometry_type == 'POSE2D':
        ros.pose2d_models.append(model.toROSPose2DModel())
      elif model.geometry_type == 'POLYGON2D':
        ros.polygon2d_models.append(model.toROSPolygon2DModel())
      elif model.geometry_type == 'POINT3D':
        ros.point3d_models.append(model.toROSPoint3DModel())
      elif model.geometry_type == 'POSE3D':
        ros.pose3d_models.append(model.toROSPose3DModel())
      elif model.geometry_type == 'POLYGON3D':
        ros.polygon3d_models.append(model.toROSPolygon3DModel())
      elif model.geometry_type == 'TRIANGLEMESH3D':
        ros.trianglemesh3d_models.append(model.toROSTriangleMesh3DModel())
      elif model.geometry_type == 'POLYGONMESH3D':
        ros.polygonmesh3d_models.append(model.toROSPolygonMesh3DModel())
      else:
        print 'ERROR: found unknown geometry type:', model.geometry_type
    return ros

""" ObjectInstance
# defines an instanciates object
# alias: gives the opportunity of well-identifiable names
# pose: gives the position and orientation of this object with respect to it's reference system
# object_description: describes the object geometrically and is used in conjunction with the pose to locate the object within it's ref_system
"""

class ObjectInstance(Base):
  __tablename__ = 'object_instance'
  id = Column('id', Integer, primary_key=True)
  alias = Column('alias', String, nullable=True)
  frame_id = Column('frame_id', Integer, ForeignKey('tree.id'), nullable=True)
  frame = relationship("FrameNode", backref=backref('object_instance', uselist=False))
  object_description_id = Column('object_description_id', Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", backref=backref('object_instance', uselist=False))

  def fromROS(self, ros):
    self.alias = ros.alias
    self.object_description_id = ros.description.id
    frame = FrameNode('','')
    frame.fromROSPoseStamped(ros.pose, self.alias)
    self.frame = frame
    return

  def toROS(self):
    ros = ROSObjectInstance()
    ros.id = self.id
    ros.alias = str(self.alias)
    ros.pose = self.frame.toROSPoseStamped()
    ros.description = self.object_description.toROS()
    return ros
