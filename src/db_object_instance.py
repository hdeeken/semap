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
from sqlalchemy.ext.hybrid import hybrid_property, hybrid_method

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
from db_geometry_model import *
from db_object_description import *
from db_transformation_tree_model import *

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

  object_description_id = Column(Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", foreign_keys=[object_description_id], backref=backref('object_instance',  uselist=True) )

  absolute_description_id = Column(Integer, ForeignKey('object_description.id'), nullable=True)
  absolute_description = relationship("ObjectDescription", foreign_keys=[absolute_description_id], backref=backref('instance') )

  def __init__(self, ros):

    if ros.alias != None:
      self.alias = ros.alias
    else:
      print 'instance without alias was created'

    if ros.description.id != None:
      self.object_description = db().query(ObjectDescription).filter(ObjectDescription.id == ros.description.id).one()
      self.name = 'object'
      
    else:
      print 'instance without desc was created'
      self.name = 'unknown'

    if ros.pose != None:
      frame = FrameNode('','')
      frame.fromROSPoseStamped(ros.pose, self.name)
      self.frame = frame
    else:
      print 'instance without frame was created'

    db().flush()
    self.name+= str(self.id)
    print 'add id', self.id
    self.name.lower()
    self.frame.name = self.name

    #if self.object_description and self.frame:
    #  print 'create INITIAL absolute'
    #  self.createAbsoluteDescription()

    return

  def getChildIDs(self):
    ids = []
    for key in self.frame.children.keys():
       child = self.frame.children[key]
       ids.append(child.object_instance.id)
    return ids

  def getAPosition2D(self):
    matrix = fromStringToMatrix(self.frame.transform)
    return WKTElement('POINT(%f %f %f)' % (matrix[0][3], matrix[1][3], 0.0))

  def getAPosition3D(self):
    matrix = fromStringToMatrix(self.frame.transform)
    return WKTElement('POINT(%f %f %f)' % (matrix[0][3], matrix[1][3], matrix[2][3]))

  def getABox2D(self):
    return self.frame.apply( self.object_description.getBox2D(as_geo = True) )

  def getABox2D2(self):
    return self.frame.apply( self.object_description.getBox2D(as_geo = True) )

  def getABox3D(self):
     return self.frame.apply( self.object_description.getBox3D(as_geo = True) )

  def getAConvexHull2D(self):
    return self.frame.apply( self.object_description.getConvexHull2D() )

  def getAConvexHull3D(self):
    return self.frame.apply( self.object_description.getConvexHull3D() )

  def toAbsoluteFootprintBoxModel(self):
    ros = Polygon2DModel()
    ros.type = "AbsoluteFootprintBox"
    ros.geometry = toPolygon2D( self.getABox2D() )
    return ros

  def toAbsoluteFootprintHullModel(self):
    ros = Polygon2DModel()
    ros.type = "AbsoluteFootprintHull"
    ros.geometry = toPolygon2D( self.getAConvexHull2D() )
    return ros

  def toAbsoluteBoundingBoxModel(self):
    ros = PolygonMesh3DModel()
    ros.type = "AbsoluteBoundingBox"
    ros.geometry = toPolygonMesh3D( self.getABox3D() )
    return ros

  def toAbsoluteBoundingHullModel(self):
    ros = PolygonMesh3DModel()
    ros.type = "AbsoluteBoundingHull"
    ros.geometry = toPolygonMesh3D( self.getAConvexHull3D() )
    return ros

  def createAbsoluteDescription(self):
    print 'would create AD'
    #self.absolute_description = ObjectDescription()
    #self.absolute_description.type = "absolute_description_" + self.name
    
    #for model in self.object_description.geometry_models:
      #new = GeometryModel()
      #new.type = model.type
      #new.pose = LocalPose()
      #new.pose.pose = fromROSPose(nullPose())
      #new.geometry_type = model.geometry_type
      #new.geometry = self.frame.apply( model.transformed() )

      #self.absolute_description.geometry_models.append(new)
      #db().add(new)

    #db().add(self.absolute_description)
    #db().commit()

    return

  def deleteAbsoluteDescription(self):
    print 'would del AD'
    #for model in self.absolute_description.geometry_models:
    #    db().delete(model.pose)
    #    db().delete(model)
    #db().delete(self.absolute_description)
    #db().commit()

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

    if self.absolute_description:
      ros.absolute = self.absolute_description.toROS()
    else:
      ros.absolute = ROSObjectDescription()

    return ros

  #ros.description.polygon2d_models.append( self.toAbsoluteFootprintBoxModel() )
  #ros.description.polygon2d_models.append( self.toAbsoluteFootprintHullModel() )

  #ros.description.polygonmesh3d_models.append( self.toAbsoluteBoundingBoxModel() )
  #ros.description.polygonmesh3d_models.append( self.toAbsoluteBoundingHullModel() )

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

  #das laeuft
  @hybrid_method
  def tester(self):
     print db().execute(WKTElement('POINT(0.0 0.0 0.0)' )).scalar()
     return db().execute(WKTElement('POINT(0.0 0.0 0.0)' )).scalar()
    #return ST_Distance( self.getABox2D(), WKTElement('POINT(1.0 0.0 0.0)') ) > 0

  @hybrid_method
  def gimme(self):
    abc = [0, 1 , 3]
    print self.frame, 'is da id'
    frame = self.frame
    print frame.id
    return abc

  @hybrid_method
  def tester2(self):
  #  matrix = fromStringToMatrix(self.frame.transform)

    a = self.gimme()[0]
    b = giveme()[1]
    c= giveme()[2]
    return  db().execute(WKTElement('POINT(%f %f %f)' % (a, b ,b))).scalar()
    #return  db().execute(WKTElement('POINT(%f %f %f)' % (matrix[0][3], matrix[1][3], 0.0)))

def giveme():
    abc = [0, 1 , 3]
    return abc
