''' Spatial DB Model

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
