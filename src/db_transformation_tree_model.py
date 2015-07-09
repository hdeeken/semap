#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from db_environment import Base, db

from sqlalchemy import Column, ForeignKey, Integer, String, create_engine
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.hybrid import hybrid_property, Comparator
from sqlalchemy.orm import relationship, backref, joinedload_all
from sqlalchemy.orm.exc import MultipleResultsFound, NoResultFound
from sqlalchemy.orm.collections import attribute_mapped_collection

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sqlalchemy import event
from sqlalchemy.event import listen
from sqlalchemy.schema import UniqueConstraint

from geometry_msgs.msg import TransformStamped, PoseStamped

from numpy import radians, dot
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, \
                               euler_from_matrix, euler_matrix, inverse_matrix, identity_matrix

def create_root_node(frame):
  try:
    # create frame

    root = FrameNode(frame, fromTransformToString([[0,0,0], [0,0,0,1]]))
    db().add(root)
    db().commit()
    return True
  except IntegrityError, e:
    db().rollback()
    print 'adding frame:', frame, 'failed'
    print 'this root frame already exists'
    print 'error:' , e
    return False

def get_root_nodes():
  roots = db().query(FrameNode.name).filter(FrameNode.parent==None).all()
  return roots

def add_transform(source_frame, target_frame, transform, append_transform = True):

  try:
    source_node = db().query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'add_transform failed to find source frame:', source_frame
    print 'error:', e
    return False

  try:
    target_node = db().query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    try:
      target_node = FrameNode(target_frame, transform, source_node)
      db().add(target_node)
      db().commit()
      return True
    except IntegrityError, e:
      db().rollback()
      print 'adding frame:', target_frame, 'failed'
      print 'the frame already exists'
      print 'error:' , e
      return False
  else:
    # update frame's transform
    if target_node.parent.name == source_frame:
      if append_transform:
        transform = fromMatrixToString(fromStringToMatrix(transform).dot(fromStringToMatrix(target_node.transform)))

      target_node.transform = transform
      db().commit()
    else:
      #update frame's source and transform
      target_node.parent = source_node
      target_node.transform = transform
      db().commit()

def change_source(source_frame, target_frame, keep_transform = False):

  try:
    source_node = db().query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'change_source failed for source:', source_frame
    print 'error:', e
    return False

  try:
    target_node = db().query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    print 'change_source failed to find target frame:', target_frame
    print 'error:' , e
    return False
  else:

    target_node.parent = source_node

    if not keep_transform:
      target_node.transform = fromTransformToString(lookup_transform(source_frame, target_frame))

    db().commit()

def remove_frame(target_frame, keep_children = False, source_frame = None):
  try:
    target_node = db().query(FrameNode).filter_by(name=target_frame).one()

    if keep_children:
      print target_node.children
      children = target_node.children
      for child in list(target_node.children):
        print child
        if source_frame != None:
          change_source(source_frame, children[child].name)
        else:
          change_source(target_node.parent.name, children[child].name)

    db().delete(target_node)
    db().commit()
    return True
  except NoResultFound, e:
    print 'removing frame:', target_frame, 'failed'
    print 'because it does not exist'
    print 'error:' , e
    return False

def lookup_transform(source_frame, target_frame):
  print 'external lookup_transform'
  try:
    source_node = db().query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'lookup_transform failed for source:', source_frame
    print 'error:', e
    return None

  try:
    target_node = db().query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    print 'lookup_transform failed for target:', target_frame
    print 'error:', e
    return None

  target_trans = target_node.root_transform
  target_matrix = fromTransformToMatrix(target_trans)
  source_trans = source_node.root_transform
  source_matrix = fromTransformToMatrix(source_trans)

  trans_matrix = inverse_matrix(source_matrix).dot(target_matrix)
  result =  fromMatrixToTransform(trans_matrix)
  return result

def rosGetTransform(source_frame, target_frame):
  transform = lookup_transform(source_frame, target_frame)
  ros = TransformStamped()
  ros.header.frame_id = source_frame
  ros.child_frame_id = target_frame
  ros.transform.translation.x = transform[0][0]
  ros.transform.translation.y = transform[0][1]
  ros.transform.translation.z = transform[0][2]
  ros.transform.rotation.x = transform[1][0]
  ros.transform.rotation.y = transform[1][1]
  ros.transform.rotation.z = transform[1][2]
  ros.transform.rotation.w = transform[1][3]
  return ros

def rosGetFrame(target_frame):
  try:
    target_node = db().query(FrameNode).filter_by(name = target_frame).one()
    transform = fromStringToTransform(target_node.transform)
    ros = TransformStamped()
    ros.header.frame_id = target_node.parent.name
    ros.child_frame_id = target_frame
    ros.transform.translation.x = transform[0][0]
    ros.transform.translation.y = transform[0][1]
    ros.transform.translation.z = transform[0][2]
    ros.transform.rotation.x = transform[1][0]
    ros.transform.rotation.y = transform[1][1]
    ros.transform.rotation.z = transform[1][2]
    ros.transform.rotation.w = transform[1][3]
    return ros
  except NoResultFound, e:
    print 'lookup_transform failed for target:', target_frame
    print 'error:', e
    return None

## ROS IO

def addROSTransformStamped(ros, append_transform = True):
  translation = [ros.transform.translation.x, \
                 ros.transform.translation.y, \
                 ros.transform.translation.z]
  rotation = [ros.transform.rotation.x, \
                 ros.transform.rotation.y, \
                 ros.transform.rotation.z, \
                 ros.transform.rotation.w]

  transform = fromTransformToString([translation, rotation])
  add_transform(ros.header.frame_id, ros.child_frame_id, transform, append_transform)

#def toROSTransformStamped():
  #ros = ROSTransformStamped()
  #ros.header.frame_id = str(self.parent.name)
  #ros.child_frame_id = str(self.name)
  #transform = fromStringToTransform(self.transform)
  #ros.transform.translation = transform[0]
  #ros.transform.rotation = transform[1]
  #return ros

## MATRIX STRING CONVERTER

'''
creates a transformation in form a 4x4 matrix
first 3 values give the x,y, offset
follwoing 9 values give the 3x3 rotation matrix
the bottom line 0, 0, 0, 1 must be set after reading
'''

def fromMatrixToString(matrix):
  string = '%f %f %f %f, %f %f %f %f, %f %f %f %f' \
  % (matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3], \
     matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3], \
     matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3])
  return string

def fromStringToMatrix(string):
  rows_ = string.split(',')
  abc_xoff = [float(x) for x in rows_[0].split()]
  def_yoff = [float(x) for x in rows_[1].split()]
  ghi_zoff = [float(x) for x in rows_[2].split()]
  matrix = [abc_xoff, def_yoff, ghi_zoff,[0,0,0,1]]
  return matrix

## DB ROS CONVERTER

def fromTransformToMatrix(transform):
  matrix = quaternion_matrix(transform[1])
  matrix[0][3] = transform[0][0]
  matrix[1][3] = transform[0][1]
  matrix[2][3] = transform[0][2]
  return matrix

def fromTransformToString(transform):
  matrix = fromTransformToMatrix(transform)
  string = fromMatrixToString(matrix)
  return string

def fromStringToTransform(string):
  matrix = fromStringToMatrix(string)
  transform = fromMatrixToTransform(matrix)
  return transform

def fromMatrixToTransform(matrix):
  quaternion = quaternion_from_matrix(matrix)
  translation = []
  translation.append(matrix[0][3])
  translation.append(matrix[1][3])
  translation.append(matrix[2][3])
  rotation = []
  rotation.append(quaternion[0])
  rotation.append(quaternion[1])
  rotation.append(quaternion[2])
  rotation.append(quaternion[3])
  return translation, rotation

## DB TF TREE
def append(transform, increment):
  update = fromTransformToMatrix(increment)
  old = fromTransformToMatrix(fromStringToTransform(transform))
  new = old.dot(update)
  return fromMatrixToString(new)

class FrameNode(Base):
    __tablename__ = 'tree'
    id = Column(Integer, primary_key=True)
    parent_id = Column(Integer, ForeignKey('tree.id'))
    name = Column(String(50), nullable=False, unique=True)
    transform = Column(String, nullable=False)

    children = relationship("FrameNode",

                        # cascade deletions
                        cascade="all, delete-orphan",

                        # many to one + adjacency list - remote_side
                        # is required to reference the 'remote'
                        # column in the join condition.
                        backref=backref("parent", remote_side=id),

                        # children will be represented as a dictionary
                        # on the "name" attribute.
                        collection_class=attribute_mapped_collection('name'),
                    )

    def __init__(self, name, transform, parent = None):
        self.name = name
        self.transform = transform
        self.parent = parent

    def __repr__(self):
      if self.parent != None:
        parent_name = self.parent.name
      else:
        parent_name = "None"

      string = "Frame(id=%r, name=%r, parent=%r tf=%r)" % (
                    self.id,
                    self.name,
                    parent_name,
                    fromStringToTransform(self.transform)
                )
      return string

    def dump(self, _indent=0):
        return "   " * _indent + repr(self) + \
                    "\n" + \
                    "".join([
                        c.dump(_indent + 1)
                        for c in self.children.values()]
                    )

    #def __after_commit_insert__(self):
    #  print 'FRAME - AFTER COMMIT - INSERT'

    #def __after_commit_update__(self):
    #  print 'FRAME - AFTER COMMIT - UPDATE'

    @hybrid_property
    def parents(self):
      allparents = []
      p = self.parent
      while p != None:
          allparents.append(p)
          p = p.parent
      return allparents

    @hybrid_property
    def path_to_root(self):
      path = []
      p = self
      while p != None:
          path.append(p)
          p = p.parent
      return path

    @hybrid_property
    def root_frame(self):
      p = self
      while p != None:
          p = p.parent
      return p

    @hybrid_property
    def root_transform(self):
      multi_matrix = identity_matrix()
      for node in self.path_to_root:
        matrix = fromStringToMatrix(node.transform)
        multi_matrix = dot(matrix, multi_matrix)
      root_transform = fromMatrixToTransform(multi_matrix)
      return root_transform

  ## FUNCTIONS

    def changeFrame(self, frame, keep_transform):
      try:
        source_node = db().query(FrameNode).filter_by(name = frame).one()
      except NoResultFound, e:
        print 'change_source failed for source:', frame
        print 'error:', e
        return False

      if not keep_transform:
        transform = self.lookup_transform(frame)
        self.transform = fromTransformToString(transform)

      self.parent = source_node

    def lookup_transform(self, frame):
      try:
        source_node = db().query(FrameNode).filter_by(name = frame).one()
      except NoResultFound, e:
        print 'change_source failed for source:', frame
        print 'error:', e
        return None

      target_trans = self.root_transform
      target_matrix = fromTransformToMatrix(target_trans)
      source_trans = source_node.root_transform
      source_matrix = fromTransformToMatrix(source_trans)
      trans_matrix = inverse_matrix(source_matrix).dot(target_matrix)
      result = fromMatrixToTransform(trans_matrix)
      return result

    def fromROSPoseStamped(self, pose, name):
      try:
        parent = db().query(FrameNode).filter_by(name = pose.header.frame_id).one()
        translation = [pose.pose.position.x, \
                     pose.pose.position.y, \
                     pose.pose.position.z]
        rotation = [pose.pose.orientation.x, \
                     pose.pose.orientation.y, \
                     pose.pose.orientation.z, \
                     pose.pose.orientation.w]

        self.name = name
        self.parent = parent
        self.transform = fromTransformToString([translation, rotation])
        return self
      except NoResultFound, e:
        print 'add_transform failed to find source frame:', pose.header.frame_id
        print 'error:', e
      return None

    def appendROSPose(self, pose):
        translation = [pose.position.x, \
                     pose.position.y, \
                     pose.position.z]
        rotation = [pose.orientation.x, \
                     pose.orientation.y, \
                     pose.orientation.z, \
                     pose.orientation.w]

        update = fromTransformToMatrix([translation, rotation])
        old = fromTransformToMatrix(fromStringToTransform(self.transform))
        new = old.dot(update)

        self.transform = fromMatrixToString(new)
        db().flush()

    def setROSPose(self, pose):

        translation = [pose.position.x, \
                     pose.position.y, \
                     pose.position.z]
        rotation = [pose.orientation.x, \
                     pose.orientation.y, \
                     pose.orientation.z, \
                     pose.orientation.w]

        self.transform = fromTransformToString([translation, rotation])
        db().flush()

    def toROSPoseStamped(self):
      transform = fromStringToTransform(self.transform)
      ros = PoseStamped()
      ros.header.frame_id = str(self.parent.name)
      ros.pose.position.x = transform[0][0]
      ros.pose.position.y = transform[0][1]
      ros.pose.position.z = transform[0][2]
      ros.pose.orientation.x = transform[1][0]
      ros.pose.orientation.y = transform[1][1]
      ros.pose.orientation.z = transform[1][2]
      ros.pose.orientation.w = transform[1][3]
      return ros

    def toROSTransformStamped(self):
      transform = fromStringToTransform(self.transform)
      ros = TransformStamped()
      ros.header.frame_id = str(self.parent.name)
      ros.child_frame_id = str(self.name)
      ros.transform.translation.x = transform[0][0]
      ros.transform.translation.y = transform[0][1]
      ros.transform.translation.z = transform[0][2]
      ros.transform.rotation.x = transform[1][0]
      ros.transform.rotation.y = transform[1][1]
      ros.transform.rotation.z = transform[1][2]
      ros.transform.rotation.w = transform[1][3]
      return ros

    def apply(self, geometry):
      matrix = fromStringToMatrix(self.transform)
      transformed_geometry = db().execute( ST_Affine( geometry, matrix[0][0], matrix[0][1], matrix[0][2], \
                                                                matrix[1][0], matrix[1][1], matrix[1][2], \
                                                                matrix[2][0], matrix[2][1], matrix[2][2], \
                                                                matrix[0][3], matrix[1][3], matrix[2][3] ) ).scalar()
      return transformed_geometry

    def apply_root_transform(self, geometry):
      matrix = fromTransformToMatrix( self.root_transform )
      transformed_geometry = db().execute( ST_Affine( geometry, matrix[0][0], matrix[0][1], matrix[0][2], \
                                                                matrix[1][0], matrix[1][1], matrix[1][2], \
                                                                matrix[2][0], matrix[2][1], matrix[2][2], \
                                                                matrix[0][3], matrix[1][3], matrix[2][3] ) ).scalar()
      return transformed_geometry
