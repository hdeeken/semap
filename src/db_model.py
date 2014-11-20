#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement

from geometry_msgs.msg import PoseStamped as ROSPose

from db_environment import Base

# GeometryModel2D
# holds the various possible 2d geometries that can be identified for an object
# model_type: defines the purpose of this geometry (e.g. 2d footprint)
# geometry_type: defines the underlying geometry (e.g. 2d polygon)
# geometry: the geometry itself as binary blob

class GeometryModel2D(Base):
  __tablename__ = 'geometry_model2d'
  id = Column('id', Integer, primary_key=True)
  model_type = Column('model_type ', String)
  geometry_type = Column('geometry_type ', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRY', dimension=2), nullable=False)

# GeometryModel3D
# holds the various possible 3d geometries that can be identified for an object
# model_type: defines the purpose of this geometry (e.g. 3d surface)
# geometry_type: defines the underlying geometry (e.g. 3d triangle mesh)
# geometry: the geometry itself as binary blob

class GeometryModel3D(Base):
  __tablename__ = 'geometry_model3d'
  id = Column('id', Integer, primary_key=True)
  model_type = Column('model_type ', String)
  geometry_type = Column('geometry_type ', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRYZ', dimension=3), nullable=False)

# ObjectDescription
# defines the geometric appearence of a object of a specific type, and is the blueprint object instances
# type: defines the type of object (e.g. Table)
# geometry_model2d: specifies all 2d geometric models bound to this object
# geometry_model3d: specifies all 2d geometric models bound to this object
# FUTURE: default model specifies which models are drawn on default

class ObjectDescription(Base):
  __tablename__ = 'object_description'
  id = Column('id', Integer, primary_key=True)
  type = Column('type', String)

## must be in instance level
##  active_model2d_id = Column('active_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
##  active_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=False))
# default is to be kept here but causes some kind of problem, needs further research
#  default_model2d_id = Column('default_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
#  default_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=False))
  geometry_model2d_id = Column('geometry_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
  geometry_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=True))
## must be in instance level
##  active_model3d_id = Column('active_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
##  active_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=False))
# default is to be kept here but causes some kind of problem, needs further research
#  default_model3d_id = Column('default_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
#  #default_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=False))
  geometry_model3d_id = Column('geometry_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
  geometry_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=True))

# ObjectInstance
# defines an instanciates object
# alias: gives the opportunity of well-identifiable names
# pose: gives the position and orientation of this object with respect to it's reference system
# object_description: describes the object geometrically and is used in conjunction with the pose to locate the object within it's ref_system

class ObjectInstance(Base):
  __tablename__ = 'object_instance'
  id = Column('id', Integer, primary_key=True)
  alias = Column('alias', String, nullable=True)
  # pose of the object
  pose_id = Column('pose_id', Integer, ForeignKey('pose.id'), nullable=True)
  pose = relationship("Pose", backref=backref('geometry_model', uselist=False))
  # object description
  object_description_id = Column('object_description_id', Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", backref=backref('object_instance', uselist=False))

class Pose(Base):
  __tablename__ = 'pose'
  id = Column('id', Integer, primary_key=True)
  ref_system = Column('ref_system', String)
  pose = Column('pose', String)

  def setPose(self, float_array):
    self.pose = ",".join(map(str, float_array))

  def getPose(self):
    return map(float, self.pose.split(','))

  def toROS(self):
    rospose = ROSPose()
    array = self.getPose()
    rospose.header.frame_id = self.ref_system
    rospose.pose.position.x = array[0]
    rospose.pose.position.y = array[1]
    rospose.pose.position.z = array[2]
    rospose.pose.orientation.x = array[3]
    rospose.pose.orientation.y = array[4]
    rospose.pose.orientation.z = array[5]
    rospose.pose.orientation.w = array[6]
    return rospose

  def fromROS(self, rospose):
      array = []
      array.append(rospose.pose.position.x)
      array.append(rospose.pose.position.y)
      array.append(rospose.pose.position.z)
      array.append(rospose.pose.orientation.x)
      array.append(rospose.pose.orientation.y)
      array.append(rospose.pose.orientation.z)
      array.append(rospose.pose.orientation.w)
      self.setPose(array)
      self.ref_system = rospose.header.frame_id
      return
