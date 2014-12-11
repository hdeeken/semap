''' Spatial DB Model

This file conjoins the different parts of the spatial database.

'''

from db_environment import *
from db_geometry_model import *
from db_object_model import *
from db_pose_model import *
from db_transformation_tree_model import *

def truncate_all():
  for table in reversed(Base.metadata.sorted_tables):
    db().execute(table.delete())

  db().commit()

def drop_all():
  Base.metadata.drop_all(engine)

def create_all():
  Base.metadata.create_all(engine)
