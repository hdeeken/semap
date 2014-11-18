#!/usr/bin/env python

from sqlalchemy import Column, Integer, String, ForeignKey
from sqlalchemy.orm import relationship

from geoalchemy2.types import Geometry
from geoalchemy2.elements import (
    WKTElement, WKBElement, RasterElement, CompositeElement
)

from db_environment import Base

class Object(Base):
  __tablename__ = 'object'
  id = Column(Integer, primary_key=True)
  of_type = Column(String)
  name = Column(String)
  pose = Column(Geometry(geometry_type='POINTZ', dimension='3'))

  def __repr__(self):
    return "<Object(id='%d' of_type='%s', name='%s')>" % (
      self.id, self.of_type, self.name)
