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

from geometry_msgs.msg import PoseStamped as ROSPose

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
  #  create_a_dummy_object_instance()
    test_object_query()
    print 'done'
