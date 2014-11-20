#!/usr/bin/env python

from sqlalchemy import Column, Integer, String
from sqlalchemy import create_engine

from db_environment import Base
from db_model import *

engine = create_engine('postgresql://hdeeken:mYdB@localhost/sfcgal_test', echo=True)
Base.metadata.create_all(engine)
