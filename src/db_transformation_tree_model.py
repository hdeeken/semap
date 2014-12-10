#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from db_environment import Base
from db_environment import Session

from sqlalchemy import Column, ForeignKey, Integer, String, create_engine
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.hybrid import hybrid_property, Comparator
from sqlalchemy.orm import relationship, backref, joinedload_all
from sqlalchemy.orm.exc import MultipleResultsFound, NoResultFound
from sqlalchemy.orm.collections import attribute_mapped_collection

from geometry_msgs.msg import TransformStamped

from numpy import radians, dot
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, \
                               euler_from_matrix, euler_matrix, inverse_matrix, identity_matrix

session = Session()

def create_root_node(frame):
  try:
    # create frame
    root = FrameNode(frame, fromTransformToString([[0,0,0], [0,0,0,1]]))
    session.add(root)
    session.commit()
    return True
  except IntegrityError, e:
    session.rollback()
    print 'adding frame:', frame, 'failed'
    print 'this root frame already exists'
    print 'error:' , e
    return False

def get_root_nodes():
  roots = session.query(FrameNode.name).filter(FrameNode.parent==None).all()
  return roots

def add_transform(source_frame, target_frame, transform):

  try:
    source_node = session.query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'add_transform failed to find source frame:', source_frame
    print 'error:', e
    return False

  try:
    target_node = session.query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    try:
      # create frame
      target_node = FrameNode(target_frame, transform, source_node)
      session.add(target_node)
      session.commit()
      return True
    except IntegrityError, e:
      print 'adding frame:', target_frame, 'failed'
      print 'the frame already exists'
      print 'error:' , e
      return False
  else:
      # update frame's transform
    if target_node.parent.name == source_frame:
      target_node.transform = transform
      session.commit()
    else:
    # update frame's source and transform
      print 'WARN update with different source frame'
      print 'old source frame:', target_node.parent.name
      print 'new source frame:', source_frame
      target_node.parent = source_node
      target_node.transform = transform
      print 'new tansform', target_node.transform
      session.commit()

def change_source(source_frame, target_frame):

  try:
    source_node = session.query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'change_source failed for source:', source_frame
    print 'error:', e
    return False

  try:
    target_node = session.query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    print 'change_source failed to find target frame:', target_frame
    print 'error:' , e
    return False
  else:
    #print 'change source for', target_frame, 'from', target_node.parent.name, 'to', source_frame
    #print 'lookup tf from', source_frame, 'to', target_frame

    new_transform = lookup_transform(source_frame, target_frame)
    target_node.parent = source_node
    target_node.transform = fromTransformToString(new_transform)
    session.commit()

def remove_frame(target_frame, keep_children = False, source_frame = None):
  try:
    target_node = session.query(FrameNode).filter_by(name=target_frame).one()

    if keep_children:
      print target_node.children
      children = target_node.children
      for child in list(target_node.children):
        print child
        if source_frame != None:
          print 'rebase', child, 'to specific:', source_frame
          change_source(source_frame, children[child].name)
        else:
          print 'rebase', child, 'to default:', target_node.parent.name
          change_source(target_node.parent.name, children[child].name)

    session.delete(target_node)
    session.commit()
    return True
  except NoResultFound, e:
    print 'removing frame:', target_frame, 'failed'
    print 'because it does not exist'
    print 'error:' , e
    return False

def lookup_transform(source_frame, target_frame):
  try:
    source_node = session.query(FrameNode).filter_by(name = source_frame).one()
  except NoResultFound, e:
    print 'lookup_transform failed for source:', source_frame
    print 'error:', e
    return None

  try:
    target_node = session.query(FrameNode).filter_by(name = target_frame).one()
  except NoResultFound, e:
    print 'lookup_transform failed for target:', target_frame
    print 'error:', e
    return None

  target_trans = target_node.root_transform
  #print 'root trans', target_node.name , target_node.root_transform
  target_matrix = fromTransformToMatrix(target_trans)

  source_trans = source_node.root_transform
  #print 'root trans', source_node.name , source_node.root_transform
  source_matrix = fromTransformToMatrix(source_trans)
  trans_matrix = target_matrix.dot(inverse_matrix(source_matrix ))

  #print 'root matrix', target_node.name, target_matrix
  #print 'root matrix', source_node.name, source_matrix
  #print 'trans_matrix', trans_matrix
  #print 'diff trans', fromMatrixToTransform(trans_matrix)
  return fromMatrixToTransform(trans_matrix)

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
    target_node = session.query(FrameNode).filter_by(name = target_frame).one()
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

def addROSTransformStamped(ros):
  translation = [ros.transform.translation.x, \
                 ros.transform.translation.y, \
                 ros.transform.translation.z]
  rotation = [ros.transform.rotation.x, \
                 ros.transform.rotation.y, \
                 ros.transform.rotation.z, \
                 ros.transform.rotation.w]

  transform = fromTransformToString([translation, rotation])
  add_transform(ros.header.frame_id, ros.child_frame_id, transform)

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
        self.parent = parent
        self.transform = transform

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

    '''
    #@hybrid_property
    #def grandparent(self):
      #if self.parent != None:
        #return self.parent.parent

    #@grandparent.comparator
    #def grandparent(cls):
        #return GrandparentTransformer(cls)
    '''

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
    def root_transform(self):
      #print 'calculating root_transform for', self.name
      multi_matrix = identity_matrix()
      for node in self.path_to_root:
        matrix = fromStringToMatrix(node.transform)
        multi_matrix = dot(multi_matrix, matrix)
        #print node.name, matrix
      root_transform = fromMatrixToTransform(multi_matrix)
     # print 'root:', root_transform
      return root_transform

  ## FUNCTIONS

    def toROS():
      transform = fromStringToTransform(self.transform)
      ros = TransformStamped()
      ros.header.frame_id = self.parent.name
      ros.child_frame_id = self.name
      ros.transform.translation.x = transform[0][0]
      ros.transform.translation.y = transform[0][1]
      ros.transform.translation.z = transform[0][2]
      ros.transform.rotation.x = transform[1][0]
      ros.transform.rotation.y = transform[1][1]
      ros.transform.rotation.z = transform[1][2]
      ros.transform.rotation.w = transform[1][3]
      return ros

    def apply(self, geometry):
      matrix = self.pose.toMatrix()
      session = Session()
      transformed_geometry = session.execute(ST_Affine(geometry, matrix[0][0], matrix[0][1], matrix[0][2], \
                                                 matrix[1][0], matrix[1][1], matrix[1][2], \
                                                 matrix[2][0], matrix[2][1], matrix[2][2], \
                                                 matrix[0][3], matrix[1][3], matrix[2][3])).scalar()
      return transformed_geometry
