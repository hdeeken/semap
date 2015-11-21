''' SEMAP DB Model

This file joins the different parts of the spatial database model
and provides methods to create, truncate or drop the entire database model.

'''

from db_environment import *
from db_pose_model import *
from db_geometry_model import *
from db_object_description import *
from db_object_instance import *
from db_transformation_tree_model import *

def create_all():
  Base.metadata.create_all(engine())

def truncate_all():
  for table in reversed(Base.metadata.sorted_tables):
    db().execute(table.delete())
  db().commit()

def drop_all():
  Base.metadata.drop_all(engine())

from sqlalchemy import MetaData
from sqlalchemy_schemadisplay import create_schema_graph

def write_graph():
  # create the pydot graph object by autoloading all tables via a bound metadata object
  graph = create_schema_graph(metadata=Base.metadata, #metadata=MetaData('postgres://user:pwd@host/database'),
     show_datatypes=False, # The image would get nasty big if we'd show the datatypes
     show_indexes=False, # ditto for indexes
     rankdir='TB',# was LR # From left to right (instead of top to bottom)
     concentrate=False # Don't try to join the relation lines together
  )
  graph.write_png('/home/hdeeken/dbschema.png') # write out the file

from sqlalchemy_schemadisplay import create_uml_graph
from sqlalchemy.orm import class_mapper

def write_uml():
  # lets find all the mappers in our model
  mappers = []
  for attr in dir(Base):
      if attr[0] == '_': continue
      try:
          cls = getattr(Base, attr)
          mappers.append(class_mapper(cls))
      except:
          pass

  # pass them to the function and set some formatting options
  graph = create_uml_graph(mappers,
      show_operations=False, # not necessary in this case
      show_multiplicity_one=False # some people like to see the ones, some don't
  )
  graph.write_png('/home/hdeeken/umlschema.png') # write out the file
