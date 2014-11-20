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
from geometry_msgs.msg import Polygon as ROSPolygon
from spatial_db.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel
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

def test_point_model_functions():

  a = Point2DModel()
  a.type = 'object_point2d_1'
  a.geometry.x = 17.0
  a.geometry.y = 4.0
  a.geometry.z = 11.0

  b = Point3DModel()
  b.type = 'object_point3d_1'
  b.geometry.x = 17.0
  b.geometry.y = 4.0
  b.geometry.z = 12.0

  c = Point3DModel()
  c.type = 'object_point3d_2'
  c.geometry.x = 19.0
  c.geometry.y = 66.0
  c.geometry.z = 12.0

  desc_ros = ROSObjectDescription()
  desc_ros.type = "first_desc"
  desc_ros.point2d_models.append(a)
  desc_ros.point3d_models.append(b)
  desc_ros.point3d_models.append(c)

  desc_db = ObjectDescription()
  desc_db.fromROS(desc_ros)

  session.add(desc_db)
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
  geo2dmodel.createFromPolygon2DModel(polygon2dmodel)

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = 'polygon3dtest'
  polygon3dmodel.geometry.points.append(point1)
  polygon3dmodel.geometry.points.append(point2)
  polygon3dmodel.geometry.points.append(point3)
  polygon3dmodel.geometry.points.append(point4)

  geo3dmodel = GeometryModel3D()
  geo3dmodel.createFromPolygon3DModel(polygon3dmodel)

  session.add(geo2dmodel)
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
       print '2D Model Type:',object1.object_description.geometry_model2d.model_type
       print '2D Geo Type:',object1.object_description.geometry_model2d.geometry_type
       print '2D Geo:', object1.object_description.geometry_model2d.geometry
       print '3D Model Type:',object1.object_description.geometry_model3d.model_type
       print '3D Geo Type::',object1.object_description.geometry_model3d.geometry_type
       print '3D Geo:', object1.object_description.geometry_model3d.geometry
       print 'Object 2'
       print 'Alias:', object2.alias
       print 'Type:',object2.object_description.type
       print '2D Model Type:',object2.object_description.geometry_model2d.model_type
       print '2D Geo Type:',object2.object_description.geometry_model2d.geometry_type
       print '2D Geo:', object2.object_description.geometry_model2d.geometry
       print '3D Model Type:',object2.object_description.geometry_model3d.model_type
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

def create_a_dummy_object_instance():
    geo2d = GeometryModel2D()
    geo2d.model_type = 'position2d'
    geo2d.geometry_type = 'POINT'
    geo2d.geometry = WKTElement('POINT(0 0)')
    geo3d = GeometryModel3D()
    geo3d.model_type = 'primitive3d'
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
  #create_a_dummy_object_instance()
  #test_object_query()
  test_point_model_functions()
  #test_polygon_model_functions()
  print 'done'
