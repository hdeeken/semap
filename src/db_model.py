#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes

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

from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix

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

  def fromPoint2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.pose = fromROSPose(nullPose())
    self.geometry_type = 'POINT2D'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, 0.0)
    self.geometry =  WKTElement(geometry_string)
    print geometry_string

  def toROSPoint2DModel(self):
    ros = Point2DModel()
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point = as_text.strip('POINT Z(').strip(')').split(' ')
    print as_text
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    ros.geometry.z = point[2]
    return ros

  def fromROSPose2DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.pose = fromMatrix(euler_matrix(0.0, 0.0, model.pose.theta))
    self.geometry_type = 'POSE2D'
    geometry_string = 'POINT(%f %f %f)' % (model.pose.position.x, model.pose.position.y, 0.0)
    self.geometry =  WKTElement(geometry_string)

  def toROSPose2DModel(self):
    ros = Point2DModel()
    ros.type = self.type
    matrix = toMatrix(self.pose.pose)
    ros.pose.x = matrix[0][3]
    ros.pose.y = matrix[1][3]
    ros.pose.theta = euler_from_matrix()[2]
    return ros

  def fromPolygon2DModel(self, model):
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
    ros.type = self.type
    ros.pose = self.pose.toROS()
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGON Z((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      values = point.split(' ')
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
    ros.type = self.type
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    point = as_text.strip('POINT Z(').strip(')').split(' ')
    ros.geometry.x = point[0]
    ros.geometry.y = point[1]
    ros.geometry.z = point[2]
    return ros

  def fromROSPose3DModel(self, model):
    self.type = model.type
    self.pose = LocalPose()
    self.pose.fromROS(model.pose)
    self.geometry_type = 'POSE3D'
    geometry_string = 'POINT(%f %f %f)' % (model.geometry.x, model.geometry.y, model.geometry.z)
    self.geometry =  WKTElement(geometry_string)

  def toROSPose3DModel(self):
    ros = Point3DModel()
    ros.type = self.type
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
    ros.type = self.type
    ros.pose = self.pose.toROS()
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygon = as_text.strip('POLYGON Z((').strip('))').split(',')
    for point in polygon[0:len(polygon)-1]:
      s = point.split(' ')
      ros.geometry.points.append(ROSPoint32(s[0],s[1],s[2]))
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
    ros.type = self.type
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
    prefix = 'POLYHEDRALSURFACEZ('
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
    ros.type = self.type
    ros.pose = self.pose.toROS()
    session = Session()
    as_text = session.execute(ST_AsText(self.geometry)).scalar()
    polygons = as_text.split(')),')
    for polygon in polygons:
      ros_polygon = ROSPolygon()
      points = polygon.strip('POLYHEDRALSURFACEZ(((').strip('))').split(',')
      for point in points[0:len(points)-1]:
        point = point.split(' ')
        ros_polygon.points.append(ROSPoint32(point[0],point[1],point[2]))
      ros.geometry.polygons.append(ros_polygon)
    return ros

### OBJECT TABLES

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
        new = GeometryModel()
        new.fromPoint2DModel(model)
        self.geometry_models.append(new)
    for model in ros.pose2d_models:
        new = GeometryModel()
        new.fromPose2DModel(model)
        self.geometry_models.append(new)
    for model in ros.polygon2d_models:
        new = GeometryModel()
        new.fromPolygon2DModel(model)
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
    return

  def toROS(self):
    ros = ROSObjectDescription()
    ros.id = self.id
    ros.type = self.type
    for model in self.geometry_models:
      if   model.geometry_type == 'POINT2D':
        ros.point2d_models.append(model.toROSPoint2DModel())
      elif model.geometry_type == 'POSE2D':
        #print 'pose2d'
        ros.pose2d_models.append(model.toROSPose2DModel())
      elif model.geometry_type == 'POLYGON2D':
        #print 'polygon2d'
        ros.polygon2d_models.append(model.toROSPolygon2DModel())
      elif model.geometry_type == 'POINT3D':
        #print 'point3d'
        ros.point3d_models.append(model.toROSPoint3DModel())
      elif model.geometry_type == 'POSE3D':
        #print 'pose3d'
        ros.point3d_models.append(model.toROSPose3DModel())
      elif model.geometry_type == 'POLYGON3D':
        #print 'polygon3d'
        ros.point3d_models.append(model.toROSPolygon3DModel())
      elif model.geometry_type == 'TRIANGLEMESH3D':
        #print 'tin3d'
        ros.point3d_models.append(model.toROSTriangleMesh3DModel())
      elif model.geometry_type == 'POLYGONMESH3D"':
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
  pose_id = Column('pose_id', Integer, ForeignKey('global_pose.id'), nullable=True)
  pose = relationship("GlobalPose", backref=backref('object_instance', uselist=False))
  # object description
  object_description_id = Column('object_description_id', Integer, ForeignKey('object_description.id'), nullable=True)
  object_description = relationship("ObjectDescription", backref=backref('object_instance', uselist=False))

  def fromROS(self, ros):
    self.alias = ros.alias
    pose = GlobalPose()
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

### POSE TABLES

class LocalPose(Base):
  __tablename__ = 'local_pose'
  id = Column('id', Integer, primary_key=True)
  pose = Column('pose', String)

  def toROS(self):
     return toROSPose(self.pose)

  def fromROS(self, ros):
    self.pose = fromROSPose(ros)
    return

class GlobalPose(Base):
  __tablename__ = 'global_pose'
  id = Column('id', Integer, primary_key=True)
  ref_system = Column('ref_system', String)
  pose = Column('pose', String)

  def toROS(self):
    ros = ROSPoseStamped()
    ros.header.frame_id = self.ref_system
    ros.pose = toROSPose(self.pose)
    return ros
  
  def fromROS(self, ros):
    self.ref_system = ros.header.frame_id
    self.pose = fromROSPose(ros.pose)

### POSE FUNCTIONS

'''
creates a transformation in form a 4x4 matrix
first 3 values give the x,y, offset
follwoing 9 values give the 3x3 rotation matrix
the bottom line 0, 0, 0, 1 must be set after reading
'''

def fromMatrix(matrix):
  string = '%f %f %f %f, %f %f %f %f, %f %f %f %f' \
  % (matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3], \
     matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3], \
     matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3])
  return string

def toMatrix(string):
  rows_ = string.split(',')
  abc_xoff = [float(x) for x in rows_[0].split()]
  def_yoff = [float(x) for x in rows_[1].split()]
  ghi_zoff = [float(x) for x in rows_[2].split()]
  matrix = [abc_xoff, def_yoff, ghi_zoff,[0,0,0,1]]
  return matrix
  
def fromROSPose(ros_pose):
  quaternion = [ros_pose.orientation.x, ros_pose.orientation.y, \
          ros_pose.orientation.z, ros_pose.orientation.w]
  matrix = quaternion_matrix(quaternion)
  matrix[0][3] = ros_pose.position.x
  matrix[0][3] = ros_pose.position.y
  matrix[0][3] = ros_pose.position.z
  return fromMatrix(matrix)

def toROSPose(db_pose):
  ros_pose = ROSPose()
  matrix = toMatrix(db_pose)
  quaternion = quaternion_from_matrix(matrix)
  ros_pose.position.x = matrix[0][3]
  ros_pose.position.y = matrix[1][3]
  ros_pose.position.z = matrix[2][3]
  ros_pose.orientation.x = quaternion[0]
  ros_pose.orientation.y = quaternion[1]
  ros_pose.orientation.z = quaternion[2]
  ros_pose.orientation.w = quaternion[3]
  return ros_pose

def nullPose():
  ros_pose = ROSPose()
  ros_pose.position.x = 0.0
  ros_pose.position.y = 0.0
  ros_pose.position.z = 0.0
  ros_pose.orientation.x = 0.0
  ros_pose.orientation.y = 0.0
  ros_pose.orientation.z = 0.0
  ros_pose.orientation.w = 1.0
  return ros_pose
