#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from geoalchemy2.elements import (
    WKTElement, WKBElement, RasterElement, CompositeElement)

from db_environment import Session
from db_model import Object

session = Session()
test_object = Object(name='hoho2', of_type='TestObject1', pose=WKTElement('POINTZ(1 0 1)'))
session.add(test_object)
session.commit()

for instance in session.query(Object).order_by(Object.of_type): 
  print instance
