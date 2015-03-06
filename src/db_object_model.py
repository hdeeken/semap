#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref
from sqlalchemy import event
from sqlalchemy.event import listen
from sqlalchemy.schema import UniqueConstraint
from sqlalchemy.orm import column_property

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sets import Set
from db_environment import *
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

  def getInstancesIDs(self):
    ids = []
    for inst in self.object_instance:
      ids.append(inst.id)
    return ids

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

  def addPoint2DModel(self, model):
    new = GeometryModel()
    new.fromROSPoint2DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPose2DModel(self, model):
    new = GeometryModel()
    new.fromROSPose2DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPolygon2DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygon2DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPoint3DModel(self, model):
    new = GeometryModel()
    new.fromROSPoint3DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPose3DModel(self, model):
    new = GeometryModel()
    new.fromROSPose3DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPolygon3DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygon3DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addTriangleMesh3DModel(self, model):
    new = GeometryModel()
    new.fromROSTriangleMesh3DModel(model)
    self.geometry_models.append(new)
    db().commit()

  def addPolygonMesh3DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygonMesh3DModel(model)
    self.geometry_models.append(new)
    db().commit()

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
  name = Column('name', String, nullable=False, unique=True)
  alias = Column('alias', String, nullable=True)
  frame_id = Column('frame_id', Integer, ForeignKey('tree.id'), nullable=True)
  frame = relationship("FrameNode", backref=backref('object_instance', uselist=False))
  object_description_id = Column('object_description_id', Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", backref=backref('object_instance', uselist=True))

  def __init__(self, ros):

    if ros.alias != None:
      self.alias = ros.alias
    else:
      print 'instance without alias was created'

    if ros.description.id != None:
      self.object_description = db().query(ObjectDescription).filter(ObjectDescription.id == ros.description.id).one()
      self.name = self.object_description.type.lower()
    else:
      print 'instance without desc was created'
      self.name = 'unknown'

    if ros.pose != None:
      frame = FrameNode('','')
      frame.fromROSPoseStamped(ros.pose, self.name)
      self.frame = frame
    else:
      print 'instance without frame was created'

    self.name+= str(self.id)
    self.name.lower()
    self.frame.name = self.name
    return

  def getChildIDs(self):
    ids = []
    for key in self.frame.children.keys():
       child = self.frame.children[key]
       ids.append(child.object_instance.id)
    return ids

  '''
    #def __before_commit_delete__(self):
    #  print 'INSTANCE - BEFORE COMMIT - DELETE'

    #def __before_commit_insert__(self):
    #  print 'INSTANCE - BEFORE COMMIT - INSERT'

    #def __before_commit_update__(self):
    #  print 'INSTANCE - BEFORE COMMIT - UPDATE'

    #def __after_commit_insert__(self):
    #  print 'INSTANCE - AFTER COMMIT - INSERT'

    #def __after_commit_update__(self):
    #  print 'INSTANCE - AFTER COMMIT - UPDATE'
  '''

  #DEPRECATED
  def fromROS(self, ros):
    self.object_description = db().query(ObjectDescription).filter(ObjectDescription.id == ros.description.id).one()
    print self.object_description
    print self.object_description.name
    self.alias = ros.alias
    frame = FrameNode('','')
    frame.fromROSPoseStamped(ros.pose, self.name)
    self.frame = frame
    return

  def toROS(self):
    ros = ROSObjectInstance()
    ros.id = self.id
    ros.name = str(self.name)
    if self.alias:
      ros.alias = str(self.alias)
    else:
      ros.alias = ""
    if self.frame:
      ros.pose = self.frame.toROSPoseStamped()
    else:
      ros.pose = ROSPoseStamped()
    if self.object_description:
      ros.description = self.object_description.toROS()
    else:
      ros.description = ROSObjectDescription()
    return ros
