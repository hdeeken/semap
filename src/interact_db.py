#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from sqlalchemy.sql import func
from sqlalchemy.orm import aliased

from geoalchemy2 import Geometry
from geoalchemy2.functions import GenericFunction
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText

from db_environment import Session
from db_model import *

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

class cos(GenericFunction):
    name = 'cos'
    type = None

class sin(GenericFunction):
    name = 'sin'
    type = None

class pi(GenericFunction):
    name = 'pi'
    type = None

class ST_3DDistance(GenericFunction):
    name = 'ST_3DDistance'
    type = None

class ST_3DIntersection(GenericFunction):
    name = 'ST_3DIntersection'
    type = None

class ST_Affine(GenericFunction):
    name = 'ST_Affine'
    type = None

session = Session()

def test_object_instance_insertion():

  point2dmodel = Point2DModel()
  point2dmodel.type = '2dpoint'
  point2dmodel.geometry.x = 17.0
  point2dmodel.geometry.y = 4.0
  point2dmodel.geometry.z = 11.0

  point3dmodel = Point3DModel()
  point3dmodel.type = '3dpoint'
  point3dmodel.geometry.x = 19.0
  point3dmodel.geometry.y = 66.0
  point3dmodel.geometry.z = 12.0

  point0 = ROSPoint32(0, 0, 0)
  point1 = ROSPoint32(1, 0, 1)
  point2 = ROSPoint32(1, 1, 0)
  point3 = ROSPoint32(0, 1, 1)
  point4 = ROSPoint32(-1, 0, 1)
  point5 = ROSPoint32(-1, 1, 1)

  polygon0 = ROSPolygon()
  polygon0.points.append(point0)
  polygon0.points.append(point1)
  polygon0.points.append(point2)
  polygon0.points.append(point3)

  polygon1 = ROSPolygon()
  polygon1.points.append(point4)
  polygon1.points.append(point0)
  polygon1.points.append(point3)
  polygon1.points.append(point5)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = '2dpolygon'
  polygon2dmodel.geometry = polygon0
  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = '3dpolygon'
  polygon3dmodel.geometry = polygon1

  tri0 = ROSMeshTriangle()
  tri1 = ROSMeshTriangle()
  tri0.vertex_indices[0] = 0
  tri0.vertex_indices[1] = 1
  tri0.vertex_indices[2] = 2

  tri1.vertex_indices[0] = 0
  tri1.vertex_indices[1] = 2
  tri1.vertex_indices[2] = 3

  trianglemesh3dmodel = TriangleMesh3DModel()
  trianglemesh3dmodel.type = '3dtrianglemesh'
  trianglemesh3dmodel.geometry.vertices.append(point0)
  trianglemesh3dmodel.geometry.vertices.append(point1)
  trianglemesh3dmodel.geometry.vertices.append(point2)
  trianglemesh3dmodel.geometry.vertices.append(point3)
  trianglemesh3dmodel.geometry.triangles.append(tri0)
  trianglemesh3dmodel.geometry.triangles.append(tri1)

  polygonmesh3dmodel = PolygonMesh3DModel()
  polygonmesh3dmodel.type = '3dpolygonmesh'
  polygonmesh3dmodel.geometry.polygons.append(polygon0)
  polygonmesh3dmodel.geometry.polygons.append(polygon1)

  desc_ros = ROSObjectDescription()
  desc_ros.type = "test_description"
  desc_ros.point2d_models.append(point2dmodel)
  desc_ros.point3d_models.append(point3dmodel)
  desc_ros.polygon2d_models.append(polygon2dmodel)
  desc_ros.polygon3d_models.append(polygon3dmodel)
  desc_ros.trianglemesh3d_models.append(trianglemesh3dmodel)
  desc_ros.polygonmesh3d_models.append(polygonmesh3dmodel)

  pose = ROSPoseStamped()
  pose.header.frame_id = "ref1"
  pose.pose.position.x = "0.0"
  pose.pose.position.y = "1.0"
  pose.pose.position.z = "0.0"
  pose.pose.orientation.x = "0.0"
  pose.pose.orientation.y = "0.0"
  pose.pose.orientation.z = "0.0"
  pose.pose.orientation.w = "1.0"

  inst_ros = ROSObjectInstance()
  inst_ros.alias = "mr_objecto"
  inst_ros.pose = pose
  inst_ros.description = desc_ros

  db_pose = Pose()
  db_pose.ref_system = 'origin'
  db_pose.pose = '1,2,1,0,0,0,1'

  # create instance from ROS
  inst_db0 = ObjectInstance()
  inst_db0.fromROS(inst_ros)
  session.add(inst_db0)

  # create instance from existing model via id
  inst_db1 = ObjectInstance()
  inst_db1.object_description_id = 1
  inst_db1.alias = "mrs_objecto"
  inst_db1.pose = db_pose
  #session.add(inst_db1)
  session.commit()

def test_get_object_instances():
  print 'hehe'
  for inst in session.query(ObjectInstance):
      desc = inst.object_description
      print 'ObjectInstance #',inst.id, 'has alias', inst.alias
      print 'Pose: ', inst.pose
      print 'ObjectDescription #',desc.id, 'has type', desc.type
      print 'and', len(desc.geometry_models2d), '2d models'
      for model in desc.geometry_models2d:
          print '    2DModel #', model.id, model.type, model.geometry_type
      print 'and', len(desc.geometry_models3d), '3d models'
      for model in desc.geometry_models3d:
          print '    3DModel #', model.id, model.type, model.geometry_type

def test_get_object_extraction():
  for db in session.query(ObjectInstance):
    instance = db.toROS()
    desc = instance.description
    print 'ObjectInstance #',instance.id, 'has alias', instance.alias
    print 'Pose: ', instance.pose
    print 'ObjectDescription #',desc.id, 'has type', desc.type
    print 'and the following 2d models:'
    for point2d in desc.point2d_models:
      print '  ', point2d.type
      print '  ', point2d.geometry
    for pose2d in desc.pose2d_models:
      print '  ', pose2d.type
      print '  ', pose2d.geometry
    for polygon2d in desc.polygon2d_models:
      print '  ', polygon2d.type
      print '  ', polygon2d.geometry
    print 'and these 3d models:'
    for point3d in desc.point3d_models:
      print '  ', point3d.type
      print '  ', point3d.geometry
    for pose3d in desc.pose3d_models:
      print '  ', pose3d.type
      print '  ', pose3d.geometry
    for polygon3d in desc.polygon3d_models:
      print '  ', polygon2d.type
      print '  ', polygon2d.geometry
    for trianglemesh3d in desc.trianglemesh3d_models:
      print '  ', trianglemesh3d.type
      print '  ', trianglemesh3d.geometry
    for polygonmesh3d in desc.polygonmesh3d_models:
      print '  ', polygonmesh3d.type
      print '  ', polygonmesh3d.geometry

def test_point_model_functions():

  a = Point3DModel()
  a.type = 'object_point2d_1'
  a.geometry.x = 17.0
  a.geometry.y = 4.0
  a.geometry.z = 11.0

  b = Point3DModel()
  b.type = 'object_point2d_2'
  b.geometry.x = 17.0
  b.geometry.y = 4.0
  b.geometry.z = 12.0

  c = Point3DModel()
  c.type = 'object_point3d_3'
  c.geometry.x = 19.0
  c.geometry.y = 66.0
  c.geometry.z = 12.0

  point1 = ROSPoint32(0, 0, 0)
  point2 = ROSPoint32(1, 0, 1)
  point3 = ROSPoint32(1, 1, 0)
  point4 = ROSPoint32(0, 1, 1)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = 'point2dtest'
  polygon2dmodel.geometry.points.append(point1)
  polygon2dmodel.geometry.points.append(point2)
  polygon2dmodel.geometry.points.append(point3)
  polygon2dmodel.geometry.points.append(point4)

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = 'polygon3dtest3'
  polygon3dmodel.geometry.points.append(point1)
  polygon3dmodel.geometry.points.append(point2)
  polygon3dmodel.geometry.points.append(point3)
  polygon3dmodel.geometry.points.append(point4)

  desc_ros = ROSObjectDescription()
  desc_ros.type = "poly_test"
  #desc_ros.point2d_models.append(a)
  #desc_ros.polygon2d_models.append(polygon2dmodel)
  #desc_ros.point2d_models.append(b)
  #desc_ros.polygon2d_models.append(polygon2dmodel)
  #desc_ros.polygon3d_models.append(polygon3dmodel)

  for i in session.query(ObjectDescription).filter(ObjectDescription.id==4):
      print i.type
      new = GeometryModel3D()
      new.fromROSPolygon3DModel(polygon3dmodel)
      i.geometry_models3d.append(new)
      session.commit()
      print 'did it'

  desc_db = ObjectDescription()
  desc_db.fromROS(desc_ros)

  #session.add(desc_db)

  for desc in session.query(ObjectDescription):
      print 'ObjectDescription #',desc.id, 'has type', desc.type
      print 'and', len(desc.geometry_models2d), '2d models'
      for model in desc.geometry_models2d:
          print '    2DModel #', model.id, model.type, model.geometry_type, model.object_description_id, model.object_description.type
      print 'and', len(desc.geometry_models3d), '3d models'
      for model in desc.geometry_models3d:
          print '    3DModel #', model.id, model.type, model.geometry_type, model.object_description_id, model.object_description.type

  session.commit()

def test_polygon_model_functions():

  point1 = ROSPoint32(0, 0, 0)
  point2 = ROSPoint32(1, 0, 1)
  point3 = ROSPoint32(1, 1, 0)
  point4 = ROSPoint32(0, 1, 1)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = 'point2dtest'
  polygon2dmodel.geometry.points.append(point1)
  polygon2dmodel.geometry.points.append(point2)
  polygon2dmodel.geometry.points.append(point3)
  polygon2dmodel.geometry.points.append(point4)

  geo2dmodel = GeometryModel2D()
  geo2dmodel.fromPolygon2DModel(polygon2dmodel)

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = 'polygon3dtest'
  polygon3dmodel.geometry.points.append(point1)
  polygon3dmodel.geometry.points.append(point2)
  polygon3dmodel.geometry.points.append(point3)
  polygon3dmodel.geometry.points.append(point4)

  geo3dmodel = GeometryModel3D()
  geo3dmodel.fromROSPolygon3DModel(polygon3dmodel)

  session.add(geo2dmodel)
  session.add(geo3dmodel)
  session.commit()

def test_mesh_model_functions():

  point1 = ROSPoint(0, 0, 0)
  point2 = ROSPoint(1, 0, 0)
  point3 = ROSPoint(1, 1, 0)
  point4 = ROSPoint(0, 1, 0)

  tri1 = ROSMeshTriangle()
  tri2 = ROSMeshTriangle()
  tri1.vertex_indices[0] = 0
  tri1.vertex_indices[1] = 1
  tri1.vertex_indices[2] = 2

  tri2.vertex_indices[0] = 0
  tri2.vertex_indices[1] = 2
  tri2.vertex_indices[2] = 3

  mesh3dmodel = TriangleMesh3DModel()
  mesh3dmodel.type = 'mesh3d'
  mesh3dmodel.geometry.vertices.append(point1)
  mesh3dmodel.geometry.vertices.append(point2)
  mesh3dmodel.geometry.vertices.append(point3)
  mesh3dmodel.geometry.vertices.append(point4)
  mesh3dmodel.geometry.triangles.append(tri1)
  mesh3dmodel.geometry.triangles.append(tri2)

  geo3dmodel = GeometryModel3D()
  geo3dmodel.fromTriangleMesh3DModel(mesh3dmodel)

  session.add(geo3dmodel)
  session.commit()

def test_object_query():
    o1 = aliased(ObjectInstance)
    o2 = aliased(ObjectInstance)
    for  object1, object2 in \
      session.query(o1, o2).\
      filter(o1.alias=='5').\
      filter(o2.alias=='6'):
       print 'Object 1'
       print 'Alias:', object1.alias
       print 'Type:',object1.object_description.type
       print '2D Model Type:',object1.object_description.geometry_model2d.type
       print '2D Geo Type:',object1.object_description.geometry_model2d.geometry_type
       print '2D Geo:', object1.object_description.geometry_model2d.geometry
       print '3D Model Type:',object1.object_description.geometry_model3d.type
       print '3D Geo Type::',object1.object_description.geometry_model3d.geometry_type
       print '3D Geo:', object1.object_description.geometry_model3d.geometry
       print 'Object 2'
       print 'Alias:', object2.alias
       print 'Type:',object2.object_description.type
       print '2D Model Type:',object2.object_description.geometry_model2d.type
       print '2D Geo Type:',object2.object_description.geometry_model2d.geometry_type
       print '2D Geo:', object2.object_description.geometry_model2d.geometry
       print '3D Model Type:',object2.object_description.geometry_model3d.type
       print '3D Geo Type::',object2.object_description.geometry_model3d.geometry_type
       print '3D Geo:', object1.object_description.geometry_model3d.geometry

       for r in session.execute(ST_AsText(ST_3DIntersection(object1.object_description.geometry_model3d.geometry, object2.object_description.geometry_model3d.geometry))):
           print r
       for r in session.execute(ST_AsText(ST_Affine(object1.object_description.geometry_model3d.geometry,  cos(pi()), -sin(pi()), 0,  sin(pi()), cos(pi()), 0,  0, 0, 1,  0, 0, 0))):
           print r
       for r in session.execute(func.ST_Distance(object1.object_description.geometry_model2d.geometry, object2.object_description.geometry_model2d.geometry)):
        print r
       for r in session.execute(func.ST_3DDistance(object1.object_description.geometry_model3d.geometry, object2.object_description.geometry_model3d.geometry)):
        print r

#sqlstring = select([regionTable], func.ST_DWithin(regionTable.c.geo_loc, 'POINT(-74.78886216922375 40.32829276931833)', 1609*50 ))

def create_a_dummy_object_instance():
    geo2d = GeometryModel2D()
    geo2d.type = 'position2d'
    geo2d.geometry_type = 'POINT'
    geo2d.geometry = WKTElement('POINT(0 0)')
    geo3d = GeometryModel3D()
    geo3d.type = 'primitive3d'
    geo3d.geometry_type = 'POINTZ'
    #geo3d.geometry_type = 'LINESTRINGZ'
    #geo3d.geometry_type = 'POLYGONZ'
    geo3d.geometry = WKTElement('POINT(0 0 0)')
    #geo3d.geometry = WKTElement('LINESTRING Z (2 2 6,1.5 1.5 7,1 1 8,0.5 0.5 8,0 0 10)')
    #geo3d.geometry = WKTElement('POLYGON((0 0 8, 0 1 8, 1 1 8, 1 0 8, 0 0 8))')

    obj_desc = ObjectDescription()
    obj_desc.type = 'test_object'
    obj_desc.geometry_model2d = geo2d
    obj_desc.geometry_model3d = geo3d

    pose = Pose()
    pose.ref_system = 'origin'
    pose.pose = '1,2,1,0,0,0,1'

    obj_inst = ObjectInstance()
    obj_inst.alias = '6'
    obj_inst.pose = pose
    obj_inst.object_description = obj_desc

    session.add(obj_inst)
    session.commit()

if __name__ == "__main__":
  #test_point_model_functions()
  #test_polygon_model_functions()
  #test_mesh_model_functions()

  #test_object_query()
  #create_a_dummy_object_instance()
  #test_object_instance_insertion()
  #test_get_object_instances()
  test_get_object_extraction()
  print 'done'
