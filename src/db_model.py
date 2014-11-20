#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement

from db_environment import Base

from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import Polygon as ROSPolygon
from spatial_db.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel
from spatial_db.msg import ObjectDescription as ROSObjectDescription
from spatial_db.msg import ObjectInstance as ROSObjectInstance

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

  def createFromPoint2DModel(self, model):
    self.model_type = model.type
    self.geometry_type = 'POINT'
    geometry_string = 'POINT(%f %f)' % (model.geometry.x, model.geometry.y)
    print geometry_string
    self.geometry =  WKTElement(geometry_string)

  def createFromPolygon2DModel(self, model):
    self.model_type = model.type
    self.geometry_type = 'POLYGON'
    prefix = 'POLYGON(('
    infix=''
    postfix = '%f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y)
    for point in model.geometry.points:
      infix = infix + '%f %f,' % ( point.x, point.y)
    print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  #def createFromPose2DModel(self, model)
  #  model_type = model.type
  #  geometry_type = 'POINT'
  #  geometry =  WKTElement('POINT(%f %f)' % model.geometry.point.x, model.geometry.point.y)

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

  def createFromPoint3DModel(self, model):
    self.model_type = model.type
    self.geometry_type = 'POINTZ'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, model.geometry.z)
    print geometry_string
    self.geometry =  WKTElement(geometry_string)

  def createFromPolygon3DModel(self, model):
    self.model_type = model.type
    self.geometry_type = 'POLYGONZ'
    prefix = 'POLYGONZ(('
    infix=''
    postfix = '%f %f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y, model.geometry.points[0].z)
    for point in model.geometry.points:
      infix = infix + '%f %f %f,' % ( point.x, point.y, point.z)
    print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

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
  #default is to be kept here but causes some kind of problem, needs further research
  #default_model2d_id = Column('default_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
  #default_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=False))
  geometry_model2d_id = Column('geometry_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
  geometry_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=True))
  #default is to be kept here but causes some kind of problem, needs further research
  #default_model3d_id = Column('default_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
  #default_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=False))
  geometry_model3d_id = Column('geometry_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
  geometry_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=True))

#string type
#string default_model
#Point2DModel[] point2d_models
#Point3DModel[] point3d_models
#Pose2DModel[] pose2d_models
#Pose3DModel[] pose3d_models
#Polygon2DModel[] polygon2d_models
#Polygon3DModel[] polygon3d_models

  def fromROS(self, ros):
    self.type = ros.type
    for model in ros.point2d_models:
        print 'got a 2d'
        self.geometry_model2d = GeometryModel2D().createFromPoint2DModel(model)
    for model in ros.point3d_models:
        print 'got a 3d'
        self.geometry_model3d = GeometryModel3D().createFromPoint3DModel(model)
    #for model in ros.pose2d_models:
    #    self.geometry_model2d = GeometryModel2D().createFromPose2DModel(model)
    #for model in ros.pose3d_models:
    #    self.geometry_model3d = GeometryModelD().createFromPose3DModel(model)
    for model in ros.polygon2d_models:
        self.geometry_model2d = GeometryModel2D().createFromPolygon2DModel(model)
    for model in ros.polygon3d_models:
        self.geometry_model3d = GeometryModel3D().createFromPolygon3DModel(model)
    return

  def toROS(self):
    ros = ROSObjectDescription()
    return ros

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
  ## must be in instance level
  ##  active_model2d_id = Column('active_model2d_id', Integer, ForeignKey('geometry_model2d.id'), nullable=True)
  ##  active_model2d = relationship("GeometryModel2D", backref=backref('object_description', uselist=False))
  ## must be in instance level
  ##  active_model3d_id = Column('active_model3d_id', Integer, ForeignKey('geometry_model3d.id'), nullable=True)
  ##  active_model3d = relationship("GeometryModel3D", backref=backref('object_description', uselist=False))

class Pose(Base):
  __tablename__ = 'pose'
  id = Column('id', Integer, primary_key=True)
  ref_system = Column('ref_system', String)
  pose = Column('pose', String)

  def setPose(self, float_array):
    self.pose = ",".join(map(str, float_array))

  def getPose(self):
    return map(float, self.pose.split(','))

  def fromROS(self, ros):
    array = []
    array.append(ros.pose.position.x)
    array.append(ros.pose.position.y)
    array.append(ros.pose.position.z)
    array.append(ros.pose.orientation.x)
    array.append(ros.pose.orientation.y)
    array.append(ros.pose.orientation.z)
    array.append(ros.pose.orientation.w)
    self.setPose(array)
    self.ref_system = ros.header.frame_id
    return

  def toROS(self):
    ros = ROSPose()
    array = self.getPose()
    ros.header.frame_id = self.ref_system
    ros.pose.position.x = array[0]
    ros.pose.position.y = array[1]
    ros.pose.position.z = array[2]
    ros.pose.orientation.x = array[3]
    ros.pose.orientation.y = array[4]
    ros.pose.orientation.z = array[5]
    ros.pose.orientation.w = array[6]
    return ros
