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

class ObjectInstance( Base ):
  __tablename__ = 'object_instance'
  id = Column( 'id', Integer, primary_key = True )
  name = Column( 'name', String, nullable = False, unique = True )
  alias = Column( 'alias', String, nullable = True )
  frame_id = Column( 'frame_id', Integer, ForeignKey( 'tree.id' ), nullable = True)
  frame = relationship( "FrameNode", backref = backref( 'object_instance', uselist = False ) )

  relative_description_id = Column( Integer, ForeignKey('object_description.id' ), nullable = True )
  relative_description = relationship( "ObjectDescription", foreign_keys = [ relative_description_id ], backref = backref( 'relative_of',  uselist=True ) )

  absolute_description_id = Column( Integer, ForeignKey( 'object_description.id' ), nullable = True)
  absolute_description = relationship( "ObjectDescription", foreign_keys = [ absolute_description_id ], backref = backref( 'absolute_of' ) )

  def __init__( self, ros ):

    if ros.alias != None:
      self.alias = ros.alias
    else:
      print 'instance without alias was created'

    if ros.description.id != None:
      self.relative_description = db().query( ObjectDescription ).filter( ObjectDescription.id == ros.description.id ).one()
      self.name = 'object'

    else:
      print 'instance without desc was created'
      self.name = 'unknown'

    if ros.pose != None:
      frame = FrameNode('','')
      frame.fromROSPoseStamped( ros.pose, self.name )
      self.frame = frame
    else:
      print 'instance without frame was created'

    db().flush()
    self.name+= str( self.id )
    self.name.lower()
    self.frame.name = self.name
    self.createAbsoluteDescription()

    return

  def getChildIDs( self ):
    ids = []
    for key in self.frame.children.keys():
       child = self.frame.children[key]
       ids.append( child.object_instance.id )
    return ids

  def getAPosition2D( self ):
    matrix = fromTransformToMatrix( self.frame.root_transform )
    return WKTElement( 'POINT(%f %f %f)' % ( matrix[0][3], matrix[1][3], 0.0) )

  def getAPosition3D( self ):
    matrix = fromTransformToMatrix( self.frame.root_transform )
    return WKTElement( 'POINT(%f %f %f)' % ( matrix[0][3], matrix[1][3], matrix[2][3] ) )

  def getABox2D( self ):
    return self.frame.apply_root_transform( self.relative_description.getBox2D( as_geo = True ) )

  def getABox3D( self ):
     return self.frame.apply_root_transform( self.relative_description.getBox3D( as_geo = True ) )

  def getAABox2D( self ):
      return db().execute( ST_Envelope( self.frame.apply_root_transform( self.relative_description.getGeometryCollection() ) ) ).scalar()

  def getAABox3D( self ):
     return box3DtoPolygonMeshGeometry( db().execute( ST_3DExtent( self.frame.apply_root_transform( self.relative_description.getGeometryCollection() ) ) ).scalar() )

  def getAConvexHull2D( self ):
    return self.frame.apply_root_transform( self.relative_description.getConvexHull2D() )

  def getAConvexHull3D( self ):
    return self.frame.apply_root_transform( self.relative_description.getConvexHull3D() )

  def getExtrudedABB2D( self ):
     return db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( self.relative_description.getBox2D( ) ), 0 ,0, 1 ) ).scalar()

  def getExtrudedACH2D( self ):
     return db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( self.relative_description.getConvexHull2D( ) ), 2 ,0, 1 ) ).scalar()

  def getExtrudedACH3D( self ):
     return db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( self.relative_description.getConvexHull3D( ) ), 2 ,0, 1 ) ).scalar()

  def toPosition2DModel( self ):
    ros = Polygon2DModel()
    ros.type = "Position2D"
    ros.geometry = toPoint2D( self.getAPosition2D() )
    return ros

  def toPosition3DModel( self ):
    ros = Polygon3DModel()
    ros.type = "Position3D"
    ros.geometry = toPoint3D( self.getAPosition3D() )
    return ros

  def toAxisAlignedFootprintBoxModel( self ):
    ros = Polygon2DModel()
    ros.type = "AxisAligned2D"
    ros.geometry = toPolygon2D( self.getAABox2D() )
    return ros

  def toAxisAlignedBoundingBoxModel( self ):
    ros = Polygon2DModel()
    ros.type = "AxisAligned3D"
    ros.geometry = toPolygonMesh3D( self.getAABox3D() )
    return ros

  def toAbsoluteFootprintBoxModel( self ):
    ros = Polygon2DModel()
    ros.type = "FootprintBox"
    ros.geometry = toPolygon2D( self.getABox2D() )
    return ros

  def toAbsoluteFootprintHullModel( self ):
    ros = Polygon2DModel()
    ros.type = "FootprintHull"
    ros.pose = nullPose()
    ros.geometry = toPolygon2D( self.getAConvexHull2D() )
    return ros

  def toAbsoluteBoundingBoxModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingBox"
    ros.geometry = toPolygonMesh3D( self.getABox3D() )
    return ros

  def toAbsoluteBoundingHullModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "BoundingHull"
    ros.geometry = toPolygonMesh3D( self.getAConvexHull3D() )
    return ros

  def toAbsoluteExtrudedBoundingBox2DModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "ExtrudedBB2D"
    ros.geometry = toPolygonMesh3D( self.getExtrudedABB2D() )
    return ros

  def toAbsoluteExtrudedConvexHull2DModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "ExtrudedCH2D"
    ros.geometry = toPolygonMesh3D( self.getExtrudedACH2D() )
    return ros

  def toAbsoluteExtrudedConvexHull3DModel( self ):
    ros = PolygonMesh3DModel()
    ros.type = "ExtrudedCH3D"
    ros.geometry = toPolygonMesh3D( self.getExtrudedACH3D() )
    return ros

  def toAbsoluteBoundingBoxFaceModels( self ):
    models = []

    faces = box3DtoBoundingBoxFaces( db().execute( ST_3DExtent( self.relative_description.getGeometryCollection() ) ).scalar() )

    print faces[0]
    ros = Polygon3DModel()
    ros.type = "Back"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[0] ) )
    models.append(ros)

    ros = Polygon3DModel()
    ros.type = "Right"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[1] ) )
    models.append(ros)

    ros = Polygon3DModel()
    ros.type = "Bottom"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[2] ) )
    models.append(ros)

    ros = Polygon3DModel()
    ros.type = "Front"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[3] ) )
    models.append(ros)

    ros = Polygon3DModel()
    ros.type = "Left"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[4] ) )
    models.append(ros)

    ros = Polygon3DModel()
    ros.type = "Top"
    ros.geometry = toPolygon3D( self.frame.apply_root_transform( faces[5] ) )
    models.append(ros)

    return models

  def toAbsoluteBoundingBoxFaceExtrusionModels( self, distance):
    models = []

    faces = box3DtoBoundingBoxFaces( db().execute( ST_3DExtent( self.relative_description.getGeometryCollection() ) ).scalar() )

    ros = PolygonMesh3DModel()
    ros.type = "BackExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(-%f 0.0 0.0)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[0] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "RightExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(0.0 -%f 0.0)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[1] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "BottomExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(0.0 0.0 -%f)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[2] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "FrontExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(%f 0.0 0.0)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[3] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "LeftExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(0.0 %f 0.0)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[4] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "TopExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(0.0 0.0 %f)' % distance) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[5] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    ros = PolygonMesh3DModel()
    ros.type = "TopLeftExtrusion"
    point = self.frame.apply_root_transform( WKTElement( 'POINT(%f %f %f)' % ( distance, distance, 0.0) ) )
    ros.geometry = toPolygonMesh3D( db().execute( SFCGAL_Extrude( self.frame.apply_root_transform( faces[2] ),
                                                                  db().execute( ST_X( point ) ).scalar() ,
                                                                  db().execute( ST_Y( point ) ).scalar() ,
                                                                  db().execute( ST_Z( point ) ).scalar() ) ).scalar() )
    models.append(ros)

    return models

  def createAbsoluteDescription( self ):

    if self.relative_description and self.frame:
      self.absolute_description = ObjectDescription()
      self.absolute_description.type = "absolute_description_" + self.name
      for model in self.relative_description.geometries:
        new = GeometryModel()
        new.type = model.type
        new.pose = LocalPose()
        new.pose.pose = fromROSPose( nullPose() )
        new.geometry_type = model.geometry_type
        new.geometry = self.frame.apply_root_transform( model.transformed() )
        self.absolute_description.geometries.append(new)

      new = GeometryModel()
      new.fromROSPoint2DModel( self.toPosition2DModel() )
      self.absolute_description.abstractions.append( new )

      new = GeometryModel()
      new.fromROSPoint3DModel( self.toPosition3DModel() )
      self.absolute_description.abstractions.append( new )

      if self.relative_description.geometries:
        new = GeometryModel()
        new.fromROSPolygon2DModel( self.toAxisAlignedFootprintBoxModel() )
        self.absolute_description.abstractions.append( new )
        
        new = GeometryModel()
        new.fromROSPolygon2DModel( self.toAbsoluteFootprintBoxModel() )
        self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygon2DModel( self.toAbsoluteFootprintHullModel() )
        self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAxisAlignedBoundingBoxModel() )
        self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAbsoluteBoundingBoxModel() )
        self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAbsoluteBoundingHullModel() )
        self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAbsoluteExtrudedConvexHull2DModel() )
        #self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAbsoluteExtrudedConvexHull3DModel() )
        #self.absolute_description.abstractions.append( new )

        new = GeometryModel()
        new.fromROSPolygonMesh3DModel( self.toAbsoluteExtrudedBoundingBox2DModel() )
        #self.absolute_description.abstractions.append( new )

        #for model in self.toAbsoluteBoundingBoxFaceModels():
        #  new = GeometryModel()
        #  new.fromROSPolygon3DModel( model )
        #  self.absolute_description.abstractions.append( new )

        #for model in self.toAbsoluteBoundingBoxFaceExtrusionModels(5.0):
        #  new = GeometryModel()
        #  new.fromROSPolygonMesh3DModel( model )
        #  self.absolute_description.abstractions.append( new )


      else:
       print 'keine geo fur absolute geos'

      db().add(self.absolute_description)
      db().commit()
    else:
      print 'cant create abs desc, either rel desc or frame is missing'
    return

  def deleteAbsoluteDescription( self ):
    if self.absolute_description:
      db().delete(self.absolute_description)
      db().commit()
    else:
      print 'no abs desc to be deleted'

  def updateAbsoluteDescription( self ):
    self.deleteAbsoluteDescription()
    self.createAbsoluteDescription()

  #DEPRECATED
  def fromROS(self, ros):
    self.relative_description = db().query(ObjectDescription).filter(ObjectDescription.id == ros.description.id).one()
    self.alias = ros.alias
    frame = FrameNode('','')
    frame.fromROSPoseStamped(ros.pose, self.name)
    self.frame = frame
    return

  def toROS( self ):
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

    if self.relative_description:
      ros.description = self.relative_description.toROS()
    else:
      ros.description = ROSObjectDescription()

    if self.absolute_description:
      ros.absolute = self.absolute_description.toROS()
    else:
      ros.absolute = ROSObjectDescription()

    return ros

  '''
    #def __before_commit_delete__( self ):
    #  print 'INSTANCE - BEFORE COMMIT - DELETE'

    #def __before_commit_insert__( self ):
    #  print 'INSTANCE - BEFORE COMMIT - INSERT'

    #def __before_commit_update__( self ):
    #  print 'INSTANCE - BEFORE COMMIT - UPDATE'

    #def __after_commit_insert__( self ):
    #  print 'INSTANCE - AFTER COMMIT - INSERT'

    #def __after_commit_update__( self ):
    #  print 'INSTANCE - AFTER COMMIT - UPDATE'
  '''

  #das laeuft
  @hybrid_method
  def tester( self ):
    print db().execute( WKTElement( 'POINT(0.0 0.0 0.0)' ) ).scalar()
    return db().execute( WKTElement('POINT(0.0 0.0 0.0)' ) ).scalar()
    #return ST_Distance( self.getABox2D(), WKTElement('POINT(1.0 0.0 0.0)') ) > 0

  @hybrid_method
  def gimme( self ):
    abc = [0, 1 , 3]
    print self.frame, 'is da id'
    frame = self.frame
    print frame.id
    return abc

  @hybrid_method
  def tester2( self ):
  #  matrix = fromStringToMatrix(self.frame.transform)

    a = self.gimme()[0]
    b = giveme()[1]
    c= giveme()[2]
    return  db().execute(WKTElement('POINT(%f %f %f)' % (a, b ,b))).scalar()
    #return  db().execute(WKTElement('POINT(%f %f %f)' % (matrix[0][3], matrix[1][3], 0.0)))

def giveme():
    abc = [0, 1 , 3]
    return abc
