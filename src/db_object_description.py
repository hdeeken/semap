#!/usr/bin/env python
import roslib; roslib.load_manifest( 'semap' )

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
from semap_msgs.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from semap_msgs.msg import ObjectDescription as ROSObjectDescription

from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix

from db_pose_model import *
from db_geometry_model import *
from db_transformation_tree_model import *

from semap.box3d_functions import *


""" ObjectDescription
# defines the geometric appearence of a object of a specific type, and is the blueprint object instances
# type: defines the type of object (e.g. Table)
# geometry_model2d: specifies all 2d geometric models bound to this object
# geometry_model3d: specifies all 2d geometric models bound to this object
# FUTURE: default model specifies which models are drawn on default
"""

class ObjectDescription(Base):
  __tablename__ = 'object_description'
  id = Column( 'id', Integer, primary_key = True )
  type = Column( 'type', String, unique = True, nullable = False )

  geometries = relationship("GeometryModel", foreign_keys='GeometryModel.geometry_desc', backref='geometry_of', cascade="save-update, merge, delete" )
  abstractions = relationship("GeometryModel", foreign_keys='GeometryModel.abstraction_desc', backref='abstracted_from', cascade="save-update, merge, delete" )

  def fromROS(self, ros):
    self.type = ros.type
    for model in ros.geometries.point2d_models:
        model = GeometryModel()
        model.fromROSPoint2DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.pose2d_models:
        model = GeometryModel()
        model.fromROSPose2DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.polygon2d_models:
        model = GeometryModel()
        model.fromROSPolygon2DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.point3d_models:
        model = GeometryModel()
        model.fromROSPoint3DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.pose3d_models:
        model = GeometryModel()
        model.fromROSPose3DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.polygon3d_models:
        model = GeometryModel()
        model.fromROSPolygon3DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.trianglemesh3d_models:
        model = GeometryModel()
        model.fromROSTriangleMesh3DModel( model )
        self.geometries.append( model )
    for model in ros.geometries.polygonmesh3d_models:
        model = GeometryModel()
        model.fromROSPolygonMesh3DModel( model )
        self.geometries.append( model )

    self.updateAbstractions()

    return

  def toROS( self ):
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = str( self.type )

    if self.geometries:
      for model in self.geometries:
        if model.geometry_type == 'POINT2D':
          ros.geometries.point2d_models.append( model.toROSPoint2DModel() )
        elif model.geometry_type == 'POSE2D':
          ros.geometries.pose2d_models.append( model.toROSPose2DModel() )
        elif model.geometry_type == 'POLYGON2D':
          ros.geometries.polygon2d_models.append( model.toROSPolygon2DModel() )
        elif model.geometry_type == 'POINT3D':
          ros.geometries.point3d_models.append( model.toROSPoint3DModel() )
        elif model.geometry_type == 'POSE3D':
          ros.geometries.pose3d_models.append( model.toROSPose3DModel() )
        elif model.geometry_type == 'POLYGON3D':
          ros.geometries.polygon3d_models.append( model.toROSPolygon3DModel() )
        elif model.geometry_type == 'TRIANGLEMESH3D':
          ros.geometries.trianglemesh3d_models.append( model.toROSTriangleMesh3DModel() )
        elif model.geometry_type == 'POLYGONMESH3D':
          ros.geometries.polygonmesh3d_models.append( model.toROSPolygonMesh3DModel() )
        else:
          print 'ERROR: found unknown geometry type:', model.geometry_type

    if self.abstractions:
      for model in self.abstractions:
        if model.geometry_type == 'POINT2D':
          ros.abstractions.point2d_models.append( model.toROSPoint2DModel() )
        elif model.geometry_type == 'POSE2D':
          ros.abstractions.pose2d_models.append( model.toROSPose2DModel() )
        elif model.geometry_type == 'POLYGON2D':
          ros.abstractions.polygon2d_models.append( model.toROSPolygon2DModel() )
        elif model.geometry_type == 'POINT3D':
          ros.abstractions.point3d_models.append( model.toROSPoint3DModel() )
        elif model.geometry_type == 'POSE3D':
          ros.abstractions.pose3d_models.append( model.toROSPose3DModel() )
        elif model.geometry_type == 'POLYGON3D':
          ros.abstractions.polygon3d_models.append( model.toROSPolygon3DModel() )
        elif model.geometry_type == 'TRIANGLEMESH3D':
          ros.abstractions.trianglemesh3d_models.append( model.toROSTriangleMesh3DModel() )
        elif model.geometry_type == 'POLYGONMESH3D':
          ros.abstractions.polygonmesh3d_models.append( model.toROSPolygonMesh3DModel() )
        else:
          print 'ERROR: found unknown geometry type:', model.geometry_type

    return ros

  def addPoint2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPoint2DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPose2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPose2DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPolygon2DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygon2DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPoint3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPoint3DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPose3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPose3DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPolygon3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygon3DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addTriangleMesh3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSTriangleMesh3DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  def addPolygonMesh3DModel( self, ros ):
    model = GeometryModel()
    model.fromROSPolygonMesh3DModel( ros )
    self.geometries.append( model )
    db().commit()
    self.updateAbstractions()
    return model.id

  # Abstractions

  def createAbstractions( self ):

    if self.geometries:
      model = GeometryModel()
      model.fromROSPolygon2DModel( self.toFootprintBoxModel() )
      self.abstractions.append( model )

      model = GeometryModel()
      model.fromROSPolygon2DModel( self.toFootprintHullModel() )
      self.abstractions.append( model )

      model = GeometryModel()
      model.fromROSPolygonMesh3DModel(self.toBoundingBoxModel())
      self.abstractions.append( model )

      model = GeometryModel()
      model.fromROSPolygonMesh3DModel(self.toBoundingHullModel())
      self.abstractions.append( model )

      #for m in self.toBoundingBoxFaceModels():
      #  model = GeometryModel()
      #  model.fromROSPolygon3DModel( m )
      #  self.abstractions.append( model )

      for m in self.toBoundingBoxExtrusionModels(5.0):
        model = GeometryModel()
        model.fromROSPolygonMesh3DModel( m )
        self.abstractions.append( model )

    else:
      print 'have no geometries'

    db().commit()

    return

  def deleteAbstractions( self ):
    for model in self.abstractions:
      db().delete( model.pose )
      db().delete( model )
    db().commit()
    return

  def updateAbstractions( self ):
    print 'update Abstractions'
    self.deleteAbstractions()
    self.createAbstractions()

  ## Getters

  def getInstancesIDs( self ):
    ids = []
    for inst in self.relative_of:
      ids.append( inst.id )
    return ids

  def getGeometryCollection( self, as_text = False ):
    models = []
    if self.geometries:
      for model in self.geometries:
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

  def toBoundingBoxFaceModels( self ):
    models = []

    for p in box3DtoBoundingBoxFaces( db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar() ):
      ros = Polygon3DModel()
      ros.type = ""
      ros.geometry = toPolygon3D( p )
      models.append(ros)

    models[0].type = "Back"
    models[1].type = "Right"
    models[2].type = "Bottom"
    models[3].type = "Front"
    models[4].type = "Left"
    models[5].type = "Top"

    return models

  def toBoundingBoxExtrusionModels(self, offset):

    models = []
    extrusions = create_extrusions( db().execute( ST_3DExtent( self.getGeometryCollection() ) ).scalar(), offset )

    ros = PolygonMesh3DModel()
    ros.type = "FrontExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[0] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontRightExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[1] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontRightTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[2] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontRightBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[3] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontLeftExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[4] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontLeftTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[5] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontLeftBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[6] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[7] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[8] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "RightExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[9] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "RightTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[10] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "RightBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[11] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "LeftExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[12] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "LeftTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[13] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "LeftBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[14] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "TopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[15] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[16] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[17] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackRightExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[18] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackRightTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[19] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackRightBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[20] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[21] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[22] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackLeftExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[23] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackLeftTopExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[24] )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BackLeftBotExtrusion"
    ros.geometry = toPolygonMesh3D( extrusions[25] )
    models.append(ros)

    return models
