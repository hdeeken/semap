#!/usr/bin/env python

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

# global declarative base class.
Base = declarative_base()
#global engine
engine = create_engine('postgresql://hdeeken:mYdB@localhost/sfcgal_test', echo=True)
#global session class
Session = sessionmaker(bind=engine) 
