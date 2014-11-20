#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from sqlalchemy.sql import func
from geoalchemy2.elements import (
    WKTElement, WKBElement, RasterElement, CompositeElement)
    
from geoalchemy2.functions import ST_Distance, ST_AsText

from db_environment import Session
from db_model import Object, Pose

from geometry_msgs.msg import PoseStamped as ROSPose

session = Session()
#session.add_all([
                 #Object(name='object1', type='Test', pose = Pose(ref_system = "map", pose_string = '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0')),
                 #Object(name='object2', type='Test', pose = Pose(ref_system = "map", pose_string = '1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0')),
                 #Object(name='object3', type='Test', pose = Pose(ref_system = "map", pose_string = '0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0')),
                 #Object(name='object4', type='Test', pose = Pose(ref_system = "map", pose_string = '0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0'))])

#session.add(Object(name='object6', type='Test', pose = Pose(ref_system = "map", pose_string = '1.0, 3.0, 4.0, 0.0, 0.0, 0.0, 1.0')))
#session.add(Object(name='object7', type='Test', pose = Pose(ref_system = "map", pose_string = '1.0, 3.0, 4.0, 0.0, 0.0, 0.0, 1.0')))
#session.commit()

for obj in session.query(Object).\
    filter(Object.type == 'Test').\
    filter(Object.geo_absolute_position!=None): 
    print obj.name
    print obj.absolute_position()

#session.commit()

#print 'skskskskskksksksksk'

##for obj in session.query(Object).filter(Object.name == 'object1'):
## print '###'
## print obj.absolute_position
## print '###'
## print obj.geo_absolute_position

#obj1 = session.query(Object).filter(Object.name == 'object1')
#obj2 = session.query(Object).filter(Object.name == 'object4')

##print obj1

##for r in session.execute(func.ST_Distance(obj1[0].geo_absolute_position, obj2[0].geo_absolute_position)):#
##	print r

from geoalchemy2 import Geometry
from geoalchemy2.functions import GenericFunction

class ST_3DDistance(GenericFunction):
    name = 'ST_3DDistance'
    type = None

from sqlalchemy.orm import aliased
o1 = aliased(Object)
o2 = aliased(Object)
#for  object1, object2, distance2D, distance3D in \
  #session.query(adalias1, adalias2, func.ST_Distance(o1.absolute_position, o2.absolute_position), func.ST_3DDistance(o1.geo_absolute_position, o2.geo_absolute_position)).\
  #filter(o1.name=='object6').\
  #filter(o2.name=='object7'):
  #print 'Object1:', object1.name
  #print 'Object2:', object2.name
  #print 'Dist2D: ', distance2D
  #print 'Dist3D: ', distance3D

#obs2 = session.query(Object).filter(Object.name == 'object6')

#for i in session.query(Object).filter(Object.name == 'object1'):
	#print i
	#print i.name
	#print i.geo_absolute_position
	#print i.absolute_position()

#print obs2[0].name, '\n'
#print obs2[0].absolute_position
#session.commit()
#print obs2[0].absolute_position

#for dist in session.execute(func.ST_Distance(obs1[0].absolute_position(), obs2[0].absolute_position())):
#    print dist

#

#for user, post in query:
#    print user.user_id, post.post_id
