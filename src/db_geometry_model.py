#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sets import Set
#from db_environment import Base
#from db_environment import Session
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

from numpy import radians
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix

#from db_object_model import *
from db_pose_model import *

### GEOMETRY TABLES

""" GeometryModel
# holds the various possible 3d geometries that can be identified for an object
# type: defines the purpose of this geometry (e.g. 3d surface)
# geometry_type: defines the underlying geometry (e.g. 3d triangle mesh)
# geometry: the geometry itself as binary blob
"""

class GeometryModel(Base):
  __tablename__ = 'geometry_model'
  id = Column('id', Integer, primary_key=True)
  type = Column('type ', String)
  object_description_id = Column('object_decription_id', Integer, ForeignKey('object_description.id'), nullable=False)
  object_description = relationship("ObjectDescription", backref=backref('geometry_models', order_by=type))
  pose_id = Column('pose_id', Integer, ForeignKey('local_pose.id'), nullable=True)
  pose = relationship("LocalPose", backref=backref('geometry_model', uselist=False))
  geometry_type = Column('geometry_type ', String)
  geometry = Column('geometry', Geometry(geometry_type='GEOMETRYZ', dimension=3), nullable=False)

  # 2D Geometry

  def fromROSPoint2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.pose = fromROSPose(nullPose())
    self.geometry_type = 'POINT2D'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, 0.0)
    self.geometry =  WKTElement(geometry_string)
    #print geometry_string

  def toROSPoint2DModel(self):
    ros = Point2DModel()
    ros.type = str(self.type)
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point = [float(x) for x in as_text.strip('POINT Z(').strip(')').split(' ')]
    #print as_text
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    ros.geometry.z = point[2]
    return ros

  def fromROSPose2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    matrix = euler_matrix(0.0, 0.0, model.pose.theta)
    matrix[0][3] = model.pose.x
    matrix[1][3] = model.pose.y
    self.pose.pose = fromMatrix(matrix)
    self.geometry_type = 'POSE2D'
    geometry_string = 'POINT(%f %f %f)' % (model.pose.x, model.pose.y, 0.0)
    self.geometry =  WKTElement(geometry_string)

  def toROSPose2DModel(self):
    ros = Pose2DModel()
    ros.type = str(self.type)
    matrix = toMatrix(self.pose.pose)
    ros.pose.x = matrix[0][3]
    ros.pose.y = matrix[1][3]
    ros.pose.theta = euler_from_matrix(matrix)[2]
    return ros

  def fromROSPolygon2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGON2D'
    prefix = 'POLYGON(('
    infix=''
    postfix = '%f %f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y, 0.0)
    for point in model.geometry.points:
      infix = infix + '%f %f %f,' % ( point.x, point.y, 0.0)
    self.geometry = WKTElement(prefix + infix + postfix)

  def toROSPolygon2DModel(self):
    ros = Polygon2DModel()
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGON Z((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      values = [float(x) for x in point.split(' ')]
      ros.geometry.points.append(ROSPoint32(values[0],values[1], 0.0))
    return ros

  # 3D Geometry

  def fromROSPoint3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(nullPose())
    self.geometry_type = 'POINT3D'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, model.geometry.z)
    self.geometry =  WKTElement(geometry_string)

  def toROSPoint3DModel(self):
    ros = Point3DModel()
    ros.type = str(self.type)
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point =  [float(x) for x in as_text.strip('POINT Z(').strip(')').split(' ')]
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    ros.geometry.z = point[2]
    return ros

  def fromROSPose3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POSE3D'
    geometry_string = 'POINT(%f %f %f)' % (model.pose.position.x, model.pose.position.y, model.pose.position.z)
    self.geometry =  WKTElement(geometry_string)

  def toROSPose3DModel(self):
    ros = Pose3DModel()
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    session = Session()
    return ros

  def fromROSPolygon3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGON3D'
    prefix = 'POLYGON(('
    infix=''
    postfix = '%f %f %f))' % ( model.geometry.points[0].x, model.geometry.points[0].y, model.geometry.points[0].z)
    for point in model.geometry.points:
      infix = infix + '%f %f %f,' % ( point.x, point.y, point.z)
    #print prefix + infix + postfix
    self.geometry = WKTElement(prefix + infix + postfix)

  def toROSPolygon3DModel(self):
    ros = Polygon3DModel()
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGON Z((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      values =  [float(x) for x in point.split(' ')]
      ros.geometry.points.append(ROSPoint32(values[0],values[1],values[2]))
    return ros

  def fromROSTriangleMesh3DModel(self, model):
    self.type = model.type
    self.geometry_type = 'TRIANGLEMESH3D'
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
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

  def toROSTriangleMesh3DModel(self):
    ros = TriangleMesh3DModel()
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
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

  def fromROSPolygonMesh3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POLYGONMESH3D'
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

  def toROSPolygonMesh3DModel(self):
    ros = PolygonMesh3DModel()
    ros.type = str(self.type)
    ros.pose = self.pose.toROS()
    ros.pose = self.pose.toROS()
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
