#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

from sqlalchemy import Column, Integer, String
from db_environment import Base
from db_model import Object 

from sqlalchemy import create_engine
engine = create_engine('postgresql://hdeeken:mYdB@localhost/sfcgal_test', echo=True)

Base.metadata.create_all(engine)
