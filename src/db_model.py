#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from geoalchemy2.shape import from_shape, to_shape

from sets import Set
from db_environment import Base
from db_environment import Session
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from spatial_db.msg import PolygonMesh as ROSPolygonMesh
from spatial_db.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db.msg import ObjectDescription as ROSObjectDescription
from spatial_db.msg import ObjectInstance as ROSObjectInstance

""" GeometryModel2D
# holds the various possible 2d geometries that can be identified for an object
# type: defines the purpose of this geometry (e.g. 2d footprint)
# geometry_type: defines the underlying geometry (e.g. 2d polygon)
# geometry: the geometry itself as binary blob
"""

class GeometryModel2D(Base):
  __tablename__ = 'geometry_model2d'
  id = Column('id', Integer, primary_key=True)
  type = Column('type', String)
  geometry_type = Column('geometry_type', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRY', dimension=2), nullable=False)
  object_description_id = Column('object_decription_id', Integer, ForeignKey('object_description.id'))
  object_description = relationship("ObjectDescription", backref=backref('geometry_models2d'))

  def fromPoint2DModel(self, model):
    self.type = model.type
    self.geometry_type = 'POINT'
    geometry_string = 'POINT(%f %f)' % (model.geometry.x, model.geometry.y)
    #print geometry_string
    self.geometry =  WKTElement(geometry_string)

  def fromPolygon2DModel(self, model):
    self.type = model.type
    self.geometry_type = 'POLYGON'
    prefix = 'POLYGON(('
    infix=''
    postfix = '%f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y)
    for point in model.geometry.points:
      infix = infix + '%f %f,' % ( point.x, point.y)
    #print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  def fromROSPose2DModel(self, model):
    print 'a pose2d feature is currently not supported'

  def toROSPoint2DModel(self):
    ros = Point2DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point = as_text.strip('POINT (').strip(')').split(' ')
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    return ros

  def toROSPose2DModel(self):
    print 'a pose2d feature is currently not supported'
    ros = Pose3DModel()
    return ros

  def toROSPolygon2DModel(self):
    ros = Polygon2DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGON ((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      values = point.split(' ')
      ros.geometry.points.append(ROSPoint32(values[0],values[1], 0.0))
    return ros

""" GeometryModel3D
# holds the various possible 3d geometries that can be identified for an object
# type: defines the purpose of this geometry (e.g. 3d surface)
# geometry_type: defines the underlying geometry (e.g. 3d triangle mesh)
# geometry: the geometry itself as binary blob
"""

class GeometryModel3D(Base):
  __tablename__ = 'geometry_model3d'
  id = Column('id', Integer, primary_key=True)
  type = Column('type ', String)
  geometry_type = Column('geometry_type ', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRYZ', dimension=3), nullable=False)
  object_description_id = Column('object_decription_id', Integer, ForeignKey('object_description.id'), nullable=False)
  object_description = relationship("ObjectDescription", backref=backref('geometry_models3d', order_by=type))

  def fromROSPoint3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'POINTZ'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, model.geometry.z)
    #print geometry_string
    self.geometry =  WKTElement(geometry_string)

  def fromROSPose3DModel(self, model):
    print 'a pose3d feature is currently not supported'

  def fromROSPolygon3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'POLYGONZ'
    prefix = 'POLYGONZ(('
    infix=''
    postfix = '%f %f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y, model.geometry.points[0].z)
    for point in model.geometry.points:
      infix = infix + '%f %f %f,' % ( point.x, point.y, point.z)
    #print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  def fromROSTriangleMesh3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'TINZ'
    prefix = 'TIN('
    postfix = ')'
    triangles = model.geometry.triangles
    vertices = model.geometry.vertices
    triangle_strings = []
    for triangle in triangles:
      indices = triangle.vertex_indices
      triangle_strings.append('((%f %f %f, %f %f %f, %f %f %f, %f %f %f))' \
              % (vertices[indices[0]].x, vertices[indices[0]].y, vertices[indices[0]].z, \
                 vertices[indices[1]].x, vertices[indices[1]].y, vertices[indices[1]].z, \
                 vertices[indices[2]].x, vertices[indices[2]].y, vertices[indices[2]].z, \
                 vertices[indices[0]].x, vertices[indices[0]].y, vertices[indices[0]].z))
    infix = ",".join(triangle_strings)
    #print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  def fromROSPolygonMesh3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'POLYHEDRALSURFACEZ'
    prefix = 'POLYHEDRALSURFACE('
    postfix = ')'
    polygons = model.geometry.polygons
    polygon_strings = []
    for polygon in polygons:
      polygon_string = ''
      for point in polygon.points:
        polygon_string = polygon_string + '%f %f %f,' % ( point.x, point.y, point.z)
      polygon_string= '(('+ polygon_string + '%f %f %f))' % ( polygon.points[0].x, polygon.points[0].y, polygon.points[0].z)
      polygon_strings.append(polygon_string)
    infix = ",".join(polygon_strings)
    #print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  def toROSPoint3DModel(self):
    ros = Point3DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point = as_text.strip('POINTZ (').strip(')').split(' ')
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    ros.geometry.z = point[2]
    return ros

  def toROSPose3DModel(self):
    print 'a pose3d feature is currently not supported'
    ros = Pose3DModel()
    return ros

  def toROSPolygon3DModel(self):
    ros = Polygon3DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGONZ ((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      s = point.split(' ')
      ros.geometry.points.append(ROSPoint32(s[0],s[1],s[2]))
    return ros

  def toROSTriangleMesh3DModel(self):
    ros = TriangleMesh3DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    triangles = as_text.split(')),')
    vertices = []
    indices = []
    ros_vertices = []
    ros_indices = []
    for triangle in triangles:
      index = []
      points = triangle.strip('TIN Z(((').strip('))').split(',')
      for point in points[0:len(points)-1]:
        point = [float(x) for x in point.split()]
        if point not in vertices:
          vertices.append(point)
          ros.geometry.vertices.append(ROSPoint(point[0],point[1],point[2]))
          index.append(vertices.index(point))
        else:
          index.append(vertices.index(point))
      indices.append(index)
      ros.geometry.triangles.append(ROSMeshTriangle(index))
    return ros

  def toROSPolygonMesh3DModel(self):
    ros = PolygonMesh3DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygons = as_text.split(')),')
    for polygon in polygons:
      ros_polygon = ROSPolygon()
      points = polygon.strip('POLYHEDRALSURFACE Z(((').strip('))').split(',')
      for point in points[0:len(points)-1]:
        point = point.split(' ')
        ros_polygon.points.append(ROSPoint32(point[0],point[1],point[2]))
      ros.geometry.polygons.append(ros_polygon)
    return ros

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

  def fromROS(self, ros):
    self.type = ros.type
    for model in ros.point2d_models:
        new = GeometryModel2D()
        new.fromPoint2DModel(model)
        self.geometry_models2d.append(new)
    for model in ros.point3d_models:
        new = GeometryModel3D()
        new.fromROSPoint3DModel(model)
        self.geometry_models3d.append(new)
    #for model in ros.pose2d_models:
    #    self.geometry_model2d = GeometryModel2D().createFromPose2DModel(model)
    #for model in ros.pose3d_models:
    #    self.geometry_model3d = GeometryModelD().createFromPose3DModel(model)
    for model in ros.polygon2d_models:
        new = GeometryModel2D()
        new.fromPolygon2DModel(model)
        self.geometry_models2d.append(new)
    for model in ros.polygon3d_models:
        new = GeometryModel3D()
        new.fromROSPolygon3DModel(model)
        self.geometry_models3d.append(new)
    for model in ros.trianglemesh3d_models:
        new = GeometryModel3D()
        new.fromROSTriangleMesh3DModel(model)
        self.geometry_models3d.append(new)
    for model in ros.polygonmesh3d_models:
        new = GeometryModel3D()
        new.fromROSPolygonMesh3DModel(model)
        self.geometry_models3d.append(new)
    return

  def toROS(self):
    print 'convert desc to ros'
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = self.type
    for model in self.geometry_models2d: # .filter(GeometryModel2D.geo_type='POINT'):
      if model.geometry_type == 'POINT':
        #print 'point2d'
        ros.point2d_models.append(model.toROSPoint2DModel())
      elif model.geometry_type == 'POSE':
        #print 'pose2d'
        ros.pose2d_models.append(model.toROSPose2DModel())
      elif model.geometry_type == 'POLYGON':
        #print 'polygon2d'
        ros.polygon2d_models.append(model.toROSPolygon2DModel())
      else:
        print 'ERROR: found unknown geometry type:', model.geometry_type

    for model in self.geometry_models3d: # .filter(GeometryModel2D.geo_type='POINT'):
      if model.geometry_type == 'POINTZ':
        #print 'point3d'
        ros.point3d_models.append(model.toROSPoint3DModel())
      elif model.geometry_type == 'POSEZ':
        #print 'pose3d'
        ros.point3d_models.append(model.toROSPose3DModel())
      elif model.geometry_type == 'POLYGONZ':
        #print 'polygon3d'
        ros.point3d_models.append(model.toROSPolygon3DModel())
      elif model.geometry_type == 'TINZ':
        #print 'tin3d'
        ros.point3d_models.append(model.toROSTriangleMesh3DModel())
      elif model.geometry_type == 'POLYHEDRALSURFACEZ':
        #print 'poly3d'
        ros.point3d_models.append(model.toROSPolygonMesh3DModel())
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
  # pose of the object
  pose_id = Column('pose_id', Integer, ForeignKey('pose.id'), nullable=True)
  pose = relationship("Pose", backref=backref('geometry_model', uselist=False))
  # object description
  object_description_id = Column('object_description_id', Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", backref=backref('object_instance', uselist=False))

  def fromROS(self, ros):
    self.alias = ros.alias
    pose = Pose()
    pose.fromROS(ros.pose)
    self.pose = pose
    description = ObjectDescription()
    description.fromROS(ros.description)
    self.object_description = description
    return

  def toROS(self):
    ros = ROSObjectInstance()
    ros.alias = self.alias
    ros.pose = self.pose.toROS()
    ros.description = self.object_description.toROS()
    return ros

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
    ros = ROSPoseStamped()
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
