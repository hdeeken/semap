#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement

from geometry_msgs.msg import PoseStamped as ROSPose

from db_environment import Base

class Pose(Base):
  __tablename__ = 'pose'
  id = Column(Integer, primary_key=True)
  ref_system = Column(String)
  pose_string = Column(String)

  def setPose(self, float_array):
    self.pose_string = ",".join(map(str, float_array))

  def getPose(self):
    return map(float, self.pose_string.split(','))

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

class Object(Base):
  __tablename__ = 'object'
  id = Column(Integer, primary_key=True)
  type = Column(String)
  name = Column(String)
  pose_id = Column(Integer, ForeignKey('pose.id'), nullable=True)
  pose = relationship("Pose", backref=backref('object', uselist=False))
  geo_absolute_position = Column(Geometry(geometry_type='POINTZ'), nullable=True) 
  # default=set_absolute_position, onupdate=set_absolute_position)

  def toROS(self):
    rosobject = ROSPose()
    rosobject.id = self.id
    rosobject.type = self.type
    rosobject.name = self.name
    rosobject.pose = self.pose.fromROS()
    return rosobject

  def fromROS(self, rosobject):
    self.type = rosobject.type
    self.name = rosobject.name
    pose = Pose()
    pose.fromROS(rosobject.pose)
    self.pose = pose
    return
  
  def __repr__(self):
    return "<Object(id='%d' type='%s', name='%s', pose_id='%d', pose='%s')>" % (
      self.id, self.type, self.name, self.pose_id, self.pose.pose_string)

  #def set_absolute_position(context):
   # self.geo_absolute_position = WKTElement("POINTZ(%f %f %f)" % (context.current_parameters['pose'].getPose()[0], context.current_parameters['pose'].getPose()[1], context.current_parameters['pose'].getPose()[2]))
