#!/usr/bin/env python
import roslib; roslib.load_manifest('semap')

import rospy
from db_environment import Base, db

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *
from operator import add
from sets import Set
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon
from mesh_msgs.msg import TriangleMesh, TriangleIndices
from mesh_msgs.msg import PolygonMesh, PolygonIndices

from numpy import radians
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix

def create_minmax(box3d, offset = 0):
  minX = db().execute( ST_XMin( box3d ) ).scalar()
  maxX = db().execute( ST_XMax( box3d ) ).scalar()
  minY = db().execute( ST_YMin( box3d ) ).scalar()
  maxY = db().execute( ST_YMax( box3d ) ).scalar()
  minZ = db().execute( ST_ZMin( box3d ) ).scalar()
  maxZ = db().execute( ST_ZMax( box3d ) ).scalar()
  #return [minX - offset, maxX + offset, minY - offset, maxY + offset, minZ - offset, maxZ + offset]
  return [minX - offset, minY - offset, minZ - offset, maxX + offset, maxY + offset, maxZ + offset]
  #0 = 0
  #1 = 3
  #2 = 1
  #3 = 4
  #4 = 2
  #5 = 5

def create_corners(minmax):

  p0 = [ minmax[0], minmax[1], minmax[2] ]
  p1 = [ minmax[0], minmax[4], minmax[2] ]
  p2 = [ minmax[0], minmax[4], minmax[5] ]
  p3 = [ minmax[0], minmax[1], minmax[5] ]

  p4 = [ minmax[3], minmax[1], minmax[2] ]
  p5 = [ minmax[3], minmax[4], minmax[2] ]
  p6 = [ minmax[3], minmax[4], minmax[5] ]
  p7 = [ minmax[3], minmax[1], minmax[5] ]

  return [p0, p1, p2, p3, p4, p5, p6, p7]

def create_faces(corners):

  f0 = [corners[0], corners[1], corners[2], corners[3]] #back
  f1 = [corners[3], corners[7], corners[4], corners[0]] #right
  f2 = [corners[0], corners[4], corners[5], corners[1]] #bottom
  f3 = [corners[7], corners[6], corners[5], corners[4]] #front
  f4 = [corners[1], corners[5], corners[6], corners[2]] #left
  f5 = [corners[2], corners[6], corners[7], corners[3]] #top

  faces = [f0, f1, f2, f3, f4, f5]

  return faces

def create_face_geometries(faces):
  prefix = 'POLYGON(('
  polygon_strings = []
  polygons = []
  for face in faces:
    polygon_string = ''
    for point in face:
      polygon_string = polygon_string + '%f %f %f,' % ( point[0], point[1], point[2])
    point =  face[0]
    polygon_string= prefix + polygon_string + '%f %f %f))' % ( point[0], point[1], point[2])
    polygons.append(WKTElement(polygon_string))
  return polygons

def create_geometry(faces):
  prefix = 'POLYHEDRALSURFACE('
  postfix = ')'
  polygon_strings = []
  for face in faces:
    polygon_string = ''
    for point in face:
      polygon_string = polygon_string + '%f %f %f,' % ( point[0], point[1], point[2])
    point =  face[0]
    polygon_string= '(('+ polygon_string + '%f %f %f))' % ( point[0], point[1], point[2])
    polygon_strings.append(polygon_string)
  infix = ",".join(polygon_strings)
  return WKTElement(prefix + infix + postfix)

def create_ros(faces):
  mesh = PolygonMesh()

  for face in faces:
    index = []
    for point in face:
      index.append(len(mesh.vertices))
      ros_point = Point()
      ros_point.x = point[0]
      ros_point.y = point[1]
      ros_point.z = point[2]
      mesh.vertices.append(ros_point)
    indices = PolygonIndices()
    indices.vertex_indices = index
    mesh.polygons.append(indices)
  return mesh

def create_extrusion_geometry(minmax):
  corners = create_corners(minmax)
  faces = create_faces(corners)
  geometry = create_geometry(faces)
  return geometry

def create_extrusions(box3d, o):
  minmax = create_minmax(box3d)
  corners = create_corners(minmax)

  x = (minmax[3] - minmax[0]) * o
  y = (minmax[4] - minmax[1]) * o
  z = (minmax[5] - minmax[2]) * o

  front_p = [sum(v) for v in zip(corners[4], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ x,  0,  0])]
  back_p  = [sum(v) for v in zip(corners[0], [-x,  0,  0])] + [sum(v) for v in zip(corners[2], [ 0,  0,  0])]
  left_p  = [sum(v) for v in zip(corners[1], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ 0,  y,  0])]
  right_p = [sum(v) for v in zip(corners[0], [ 0, -y,  0])] + [sum(v) for v in zip(corners[7], [ 0,  0,  0])]
  top_p   = [sum(v) for v in zip(corners[3], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ 0,  0,  z])]
  bot_p   = [sum(v) for v in zip(corners[0], [ 0,  0, -z])] + [sum(v) for v in zip(corners[5], [ 0,  0,  0])]

  front_hs = [sum(v) for v in zip(corners[4], [ 0, -y, -z])] + [sum(v) for v in zip(corners[6], [ x,  y,  z])]
  back_hs  = [sum(v) for v in zip(corners[0], [-x, -y, -z])] + [sum(v) for v in zip(corners[2], [ 0,  y,  z])]
  left_hs  = [sum(v) for v in zip(corners[1], [-x,  0, -z])] + [sum(v) for v in zip(corners[6], [ x,  y,  z])]
  right_hs = [sum(v) for v in zip(corners[0], [-x, -y, -z])] + [sum(v) for v in zip(corners[7], [ x,  0,  z])]
  top_hs   = [sum(v) for v in zip(corners[3], [-x, -y,  0])] + [sum(v) for v in zip(corners[6], [ x,  y,  z])]
  bot_hs   = [sum(v) for v in zip(corners[0], [-x, -y, -z])] + [sum(v) for v in zip(corners[5], [ x,  y,  0])]

  geometries = []
  geometries.append( create_extrusion_geometry( front_p ) )
  geometries.append( create_extrusion_geometry( back_p ) )
  geometries.append( create_extrusion_geometry( left_p ) )
  geometries.append( create_extrusion_geometry( right_p ) )
  geometries.append( create_extrusion_geometry( top_p ) )
  geometries.append( create_extrusion_geometry( bot_p ) )
  geometries.append( create_extrusion_geometry( front_hs ) )
  geometries.append( create_extrusion_geometry( back_hs ) )
  geometries.append( create_extrusion_geometry( left_hs ) )
  geometries.append( create_extrusion_geometry( right_hs ) )
  geometries.append( create_extrusion_geometry( top_hs ) )
  geometries.append( create_extrusion_geometry( bot_hs ) )

  '''
  front_right     = [sum(v) for v in zip(corners[4], [ 0, -y,  0])] + [sum(v) for v in zip(corners[7], [ x,  0,  0])]
  front_right_top = [sum(v) for v in zip(corners[7], [ 0, -y,  0])] + [sum(v) for v in zip(corners[7], [ x,  0,  z])]
  front_right_bot = [sum(v) for v in zip(corners[4], [ 0, -y, -z])] + [sum(v) for v in zip(corners[4], [ x,  0,  0])]
  front_left      = [sum(v) for v in zip(corners[5], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ x,  y,  0])]
  front_left_top  = [sum(v) for v in zip(corners[6], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ x,  y,  z])]
  front_left_bot  = [sum(v) for v in zip(corners[5], [ 0,  0, -z])] + [sum(v) for v in zip(corners[5], [ x,  y,  0])]
  front_top       = [sum(v) for v in zip(corners[7], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ x,  0,  z])]
  front_bot       = [sum(v) for v in zip(corners[4], [ 0,  0, -z])] + [sum(v) for v in zip(corners[5], [ x,  0,  0])]
  right_top       = [sum(v) for v in zip(corners[3], [ 0, -y,  0])] + [sum(v) for v in zip(corners[7], [ 0,  0,  z])]
  right_bot       = [sum(v) for v in zip(corners[0], [ 0, -y, -z])] + [sum(v) for v in zip(corners[4], [ 0,  0,  0])]
  left_top        = [sum(v) for v in zip(corners[2], [ 0,  0,  0])] + [sum(v) for v in zip(corners[6], [ 0,  y,  z])]
  left_bot        = [sum(v) for v in zip(corners[1], [ 0,  0, -z])] + [sum(v) for v in zip(corners[5], [ 0,  y,  0])]
  back_right      = [sum(v) for v in zip(corners[0], [-x, -y,  0])] + [sum(v) for v in zip(corners[3], [ 0,  0,  0])]
  back_right_top  = [sum(v) for v in zip(corners[3], [-x, -y,  0])] + [sum(v) for v in zip(corners[3], [ 0,  0,  z])]
  back_right_bot  = [sum(v) for v in zip(corners[0], [-x, -y, -z])] + [sum(v) for v in zip(corners[0], [ 0,  0,  0])]
  back_top        = [sum(v) for v in zip(corners[3], [-x,  0,  0])] + [sum(v) for v in zip(corners[2], [ 0,  0,  z])]
  back_bot        = [sum(v) for v in zip(corners[0], [-x,  0, -z])] + [sum(v) for v in zip(corners[1], [ 0,  0,  0])]
  back_left       = [sum(v) for v in zip(corners[1], [-x,  0,  0])] + [sum(v) for v in zip(corners[2], [ 0,  y,  0])]
  back_left_top   = [sum(v) for v in zip(corners[2], [-x,  0,  0])] + [sum(v) for v in zip(corners[2], [ 0,  y,  z])]
  back_left_bot   = [sum(v) for v in zip(corners[1], [-x,  0, -z])] + [sum(v) for v in zip(corners[1], [ 0,  y,  0])]
  geometries.append( create_extrusion_geometry( front_right ) )
  geometries.append( create_extrusion_geometry( front_right_top ) )
  geometries.append( create_extrusion_geometry( front_right_bot ) )
  geometries.append( create_extrusion_geometry( front_left ) )
  geometries.append( create_extrusion_geometry( front_left_top ) )
  geometries.append( create_extrusion_geometry( front_left_bot ) )
  geometries.append( create_extrusion_geometry( front_top ) )
  geometries.append( create_extrusion_geometry( front_bot ) )
  geometries.append( create_extrusion_geometry( right_top ) )
  geometries.append( create_extrusion_geometry( right_bot ) )
  geometries.append( create_extrusion_geometry( left_top ) )
  geometries.append( create_extrusion_geometry( left_bot ) )
  geometries.append( create_extrusion_geometry( back_right ) )
  geometries.append( create_extrusion_geometry( back_right_top ) )
  geometries.append( create_extrusion_geometry( back_right_bot ) )
  geometries.append( create_extrusion_geometry( back_top ) )
  geometries.append( create_extrusion_geometry( back_bot ) )
  geometries.append( create_extrusion_geometry( back_left ) )
  geometries.append( create_extrusion_geometry( back_left_top ) )
  geometries.append( create_extrusion_geometry( back_left_bot ) )
  '''

  return geometries

## Production Functions

def box3DtoPolygonMesh(box3d, offset = 0):

  minmax = create_minmax(box3d, offset)
  corners = create_corners(minmax)
  faces = create_faces(corners)
  mesh = create_ros(faces)

  return mesh

def box3DtoPolygonMeshGeometry(box3d, offset = 0):

  minmax = create_minmax(box3d, offset)
  corners = create_corners(minmax)
  faces = create_faces(corners)
  geometry = create_geometry(faces)

  return geometry

def box3DtoBoundingBoxFaces(box3d, offset = 0):

  minmax = create_minmax(box3d, offset)
  corners = create_corners(minmax)
  faces = create_faces(corners)
  geometries = create_face_geometries(faces)

  return geometries

