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
from db_transformation_tree_model import *

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

    self.updateAbstractions()

    return

  def createAbstractions(self):
    print 'create abstractions of', self.type
    if self.geometry_models:
      new = GeometryModel()
      new.fromROSPolygon2DModel(self.toFootprintBoxModel())
      self.geometry_models.append(new)

      print self.geometry_models
      db().flush()

      new = GeometryModel()
      new.fromROSPolygon2DModel(self.toFootprintHullModel())
      self.geometry_models.append(new)
      db().flush()

      #new = GeometryModel()
      #new.fromROSPolygonMesh3DModel(self.toBoundingBoxModel())
      #self.geometry_models.append(new)

      #new = GeometryModel()
      #new.fromROSPolygonMesh3DModel(self.toBoundingHullModel())
      #self.geometry_models.append(new)

    db().commit()
    return

  def deleteAbstractions(self):
    print 'delete abstractions of', self.type
    for model in self.geometry_models:
      if model.type in ["FootprintBox", "FootprintHull", "BoundingBox", "BoundingHull"]:
        print 'delete', model.type
        db().delete(model.pose)
        db().delete(model)
    db().commit()
    return

  def updateAbstractions(self):
    print 'update abstractions of', self.type
    self.deleteAbstractions()
    self.createAbstractions()

  def addPoint2DModel(self, model):
    new = GeometryModel()
    new.fromROSPoint2DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPose2DModel(self, model):
    new = GeometryModel()
    new.fromROSPose2DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPolygon2DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygon2DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPoint3DModel(self, model):
    new = GeometryModel()
    new.fromROSPoint3DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPose3DModel(self, model):
    new = GeometryModel()
    new.fromROSPose3DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPolygon3DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygon3DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addTriangleMesh3DModel(self, model):
    new = GeometryModel()
    new.fromROSTriangleMesh3DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def addPolygonMesh3DModel(self, model):
    new = GeometryModel()
    new.fromROSPolygonMesh3DModel(model)
    self.geometry_models.append(new)
    db().commit()
    self.updateAbstractions()

  def toROS(self):
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = str(self.type)

    if self.geometry_models:

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

      #experimental: concave hulls
      #ros.polygon2d_models.append( self.toFootprintHullModel() )

    return ros

  def getBody(self, as_text = False):
    for model in self.geometry_models:
      if model.type == "Body":
        return model.transformed()

  def getGeometryCollection(self, as_text = False):
    models = []
    for model in self.geometry_models:
      if model.type not in ["FootprintBox", "FootprintHull", "BoundingBox", "BoundingHull"]:
        models.append( model.transformed() )
      if not as_text:
        return db().execute( ST_Collect(models) ).scalar()
    else:
      return db().execute( ST_AsText( ST_Collect(models) ) ).scalar()

  def getBox2D(self, as_geo = True):
    if as_geo:
      return db().execute( ST_Envelope( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_Extent( self.getGeometryCollection() ) ).scalar()

  def getBox3D(self, as_geo = True):
    if as_geo:
      return box3DtoPolygonMeshGeometry( db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar() )
    else:
      return db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar()

  def getConvexHull2D(self, as_text = False):
    if not as_text:
      return db().execute( SFCGAL_Convexhull( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_AsText( SFCGAL_Convexhull( self.getGeometryCollection(True) ) ) ).scalar()

  def getConvexHull3D(self, as_text = False):
    if not as_text:
      return db().execute( SFCGAL_Convexhull3D( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_AsText( SFCGAL_Convexhull3D( self.getGeometryCollection(True) ) ) ).scalar()

  # probably DOES NOT WORK WITH TIN debug further
  def getConcaveHull2D(self, as_text = False):
    if not as_text:
      return db().execute( ST_ConcaveHull( self.getGeometryCollection() , 0.8) ).scalar()
    else:
      return db().execute( ST_AsText( ST_ConcaveHull( self.getGeometryCollection() , 0.8) ) ).scalar()

  def toBoundingBoxModel(self):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingBox"
    ros.geometry = toPolygonMesh3D( self.getBox3D() )
    return ros

  def toBoundingHullModel(self):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingHull"
    ros.geometry = toPolygonMesh3D( self.getConvexHull3D() )
    return ros

  def toFootprintBoxModel(self):
    ros = Polygon2DModel()
    ros.type = "FootprintBox"
    geo = self.getBox2D()
    ros.geometry = toPolygon2D( geo )
    return ros

  def toFootprintHullModel(self):
    ros = Polygon2DModel()
    ros.type = "FootprintHull"
    ros.geometry = toPolygon2D( self.getConvexHull2D() )
    return ros

  def toFootprintHull2Model(self):
    ros = Polygon2DModel()
    ros.type = "FootprintHull2"
    ros.geometry = toPolygon2D( self.getConcaveHull2D() )
    return ros

# geometry ST_Envelope(geometry g1);
# Returns the float4 minimum bounding box for the supplied geometry, as a geometry.
# The polygon is defined by the corner points of the bounding box ((MINX, MINY), (MINX, MAXY), (MAXX, MAXY), (MAXX, MINY), (MINX, MINY)). (PostGIS will add a ZMIN/ZMAX coordinate as well).
#POLYGON((-0.052632 -0.052343,-0.052632 0.052343,0.080814 0.052343,0.080814 -0.052343,-0.052632 -0.052343))

#class RelativeDescription(ObjectDescription, Base):
    #__tablename__ = 'relative_description'

#class AbsoluteDescription(ObjectDescription, Base):
    #__tablename__ = 'absolute_description'
