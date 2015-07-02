#!/usr/bin/env python
import roslib; roslib.load_manifest( 'spatial_db' )

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

from db_environment import *
from spatial_db_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db_msgs.msg import ObjectDescription as ROSObjectDescription

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
  id = Column( 'id', Integer, primary_key=True )
  type = Column( 'type', String )
  #geometry_id = Column(Integer, ForeignKey('geometry_model.id'))
  geometries = relationship("GeometryModel")
  abstractions = relationship("GeometryModel")
  ## Constructors / Destructors

class Parent(Base):
    __tablename__ = 'parent'
    id = Column(Integer, primary_key=True)
    children = relationship("Child")

class Child(Base):
    __tablename__ = 'child'
    id = Column(Integer, primary_key=True)




  def fromROS(self, ros):
    self.type = ros.type
    for model in ros.point2d_models:
        model = GeometryModel()
        model.fromROSPoint2DModel( model )
        self.geometry_models.append( model )
    for model in ros.pose2d_models:
        model = GeometryModel()
        model.fromROSPose2DModel( model )
        self.geometry_models.append( model )
    for model in ros.polygon2d_models:
        model = GeometryModel()
        model.fromROSPolygon2DModel( model )
        self.geometry_models.append( model )
    for model in ros.point3d_models:
        model = GeometryModel()
        model.fromROSPoint3DModel( model )
        self.geometry_models.append( model )
    for model in ros.pose3d_models:
        model = GeometryModel()
        model.fromROSPose3DModel( model )
        self.geometry_models.append( model )
    for model in ros.polygon3d_models:
        model = GeometryModel()
        model.fromROSPolygon3DModel( model )
        self.geometry_models.append( model )
    for model in ros.trianglemesh3d_models:
        model = GeometryModel()
        model.fromROSTriangleMesh3DModel( model )
        self.geometry_models.append( model )
    for model in ros.polygonmesh3d_models:
        model = GeometryModel()
        model.fromROSPolygonMesh3DModel( model )
        self.geometry_models.append( model )

    self.updateAbstractions()

    return

  def toROS( self ):
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = str( self.type )

    if self.geometry_models:
      for model in self.geometry_models:
        if model.geometry_type == 'POINT2D':
          ros.point2d_models.append( model.toROSPoint2DModel() )
        elif model.geometry_type == 'POSE2D':
          ros.pose2d_models.append( model.toROSPose2DModel() )
        elif model.geometry_type == 'POLYGON2D':
          ros.polygon2d_models.append( model.toROSPolygon2DModel() )
        elif model.geometry_type == 'POINT3D':
          ros.point3d_models.append( model.toROSPoint3DModel() )
        elif model.geometry_type == 'POSE3D':
          ros.pose3d_models.append( model.toROSPose3DModel() )
        elif model.geometry_type == 'POLYGON3D':
          ros.polygon3d_models.append( model.toROSPolygon3DModel() )
        elif model.geometry_type == 'TRIANGLEMESH3D':
          ros.trianglemesh3d_models.append( model.toROSTriangleMesh3DModel() )
        elif model.geometry_type == 'POLYGONMESH3D':
          ros.polygonmesh3d_models.append( model.toROSPolygonMesh3DModel() )
        else:
          print 'ERROR: found unknown geometry type:', model.geometry_type

    return ros

  ## addGeometry

  def addPoint2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPoint2DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPose2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPose2DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPolygon2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygon2DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPoint3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPoint3DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPose3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPose3DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPolygon3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygon3DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addTriangleMesh3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSTriangleMesh3DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  def addPolygonMesh3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygonMesh3DModel( ros )
    self.geometry_models.append( model )
    db().commit()
    self.updateAbstractions()

  # Abstractions

  def createAbstractions( self ):
    if self.geometry_models:
      model = GeometryModel()
      model.fromROSPolygon2DModel( self.toFootprintBoxModel() )
      self.geometry_models.append( model )
      db().flush()
      model = GeometryModel()
      model.fromROSPolygon2DModel( self.toFootprintHullModel() )
      self.geometry_models.append( model )
      db().flush()
      #model = GeometryModel()
      #model.fromROSPolygonMesh3DModel(self.toBoundingBoxModel())
      #self.geometry_models.append( model )
      #model = GeometryModel()
      #model.fromROSPolygonMesh3DModel(self.toBoundingHullModel())
      #self.geometry_models.append( model )
    db().commit()
    return

  def deleteAbstractions( self ):
    print 'delete abstractions of', self.type
    for model in self.geometry_models:
      if model.type in [ "FootprintBox", "FootprintHull", "BoundingBox", "BoundingHull" ]:
        db().delete( model.pose )
        db().delete( model )
    db().commit()
    return

  def updateAbstractions( self ):
    print 'update abstractions of', self.type
    self.deleteAbstractions()
    self.createAbstractions()

  ## Getters

  def getInstancesIDs( self ):
    ids = []
    for inst in self.object_instance:
      ids.append( inst.id )
    return ids

  def getBody( self, as_text = False ):
    for model in self.geometry_models:
      if model.type == "Body":
        return model.transformed()

  def getGeometryCollection( self, as_text = False ):
    models = []
    for model in self.geometry_models:
      if model.type not in [ "FootprintBox", "FootprintHull", "BoundingBox", "BoundingHull" ]:
        models.append( model.transformed() )
      if not as_text:
        return db().execute( ST_Collect( models ) ).scalar()
    else:
      return db().execute( ST_AsText( ST_Collect( models ) ) ).scalar()

  def getBox2D( self, as_geo = True ):
    if as_geo:
      return db().execute( ST_Envelope( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_Extent( self.getGeometryCollection() ) ).scalar()

  def getBox3D( self, as_geo = True ):
    if as_geo:
      return box3DtoPolygonMeshGeometry( db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar() )
    else:
      return db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar()

  def getConvexHull2D( self, as_text = False ):
    if not as_text:
      return db().execute( SFCGAL_Convexhull( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_AsText( SFCGAL_Convexhull( self.getGeometryCollection( True ) ) ) ).scalar()

  def getConvexHull3D( self, as_text = False):
    if not as_text:
      return db().execute( SFCGAL_Convexhull3D( self.getGeometryCollection() ) ).scalar()
    else:
      return db().execute( ST_AsText( SFCGAL_Convexhull3D( self.getGeometryCollection( True ) ) ) ).scalar()

  def toBoundingBoxModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingBox"
    ros.geometry = toPolygonMesh3D( self.getBox3D() )
    return ros

  def toBoundingHullModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingHull"
    ros.geometry = toPolygonMesh3D( self.getConvexHull3D() )
    return ros

  def toFootprintBoxModel( self ):
    ros = Polygon2DModel()
    ros.type = "FootprintBox"
    geo = self.getBox2D()
    ros.geometry = toPolygon2D( geo )
    return ros

  def toFootprintHullModel( self ):
    ros = Polygon2DModel()
    ros.type = "FootprintHull"
    ros.geometry = toPolygon2D( self.getConvexHull2D() )
    return ros

#class RelativeDescription( ObjectDescription, Base ):
    #__tablename__ = 'relative_description'

#class AbsoluteDescription( ObjectDescription, Base ):
    #__tablename__ = 'absolute_description'
