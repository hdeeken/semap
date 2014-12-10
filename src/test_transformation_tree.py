#!/usr/bin/env python

from db_environment import Base
from db_environment import Session
from db_model import create_all, drop_all

from db_transformation_tree_model import *
from sqlalchemy.orm import aliased

from sqlalchemy import Column, ForeignKey, Integer, String, func
from sqlalchemy.orm import relationship, backref, joinedload_all

from sqlalchemy.orm.collections import attribute_mapped_collection

def msg(msg, *args):
  msg = msg % args
  print("\n\n\n" + "-" * len(msg.split("\n")[0]))
  print(msg)
  print("-" * len(msg.split("\n")[0]))

def create_tree():
  msg("Creating Tree Table:")

  create_root_node('world')

  test_ros_insert('world', 'floor0', [0,0,0], [0,0,0,1])
  test_ros_insert('world', 'floor1', [0,0,5], [0,0,0,1])

  test_ros_insert('floor0', 'room0', [0,0,0], [0,0,0,1])
  test_ros_insert('floor1', 'room1', [0,0,0], [0,0,0,1])

  test_ros_insert('room0', 'table0', [0,1,1], [0,0,0,1])
  test_ros_insert('room1', 'table1', [1,0,0], [0,0,0,1])

  test_ros_insert('room0', 'mug0', [1,0,1], [0,0,0,1])
  test_ros_insert('table1', 'mug1', [0,0,1], [0,0,0,1])

def test_ros_insert(source_frame, target_frame, translation, rotation):
  tf = TransformStamped()
  tf.header.frame_id = source_frame
  tf.child_frame_id = target_frame
  tf.transform.translation.x = translation[0]
  tf.transform.translation.y = translation[1]
  tf.transform.translation.z = translation[2]
  tf.transform.rotation.x = rotation[0]
  tf.transform.rotation.y = rotation[1]
  tf.transform.rotation.z = rotation[2]
  tf.transform.rotation.w = rotation[3]
  addROSTransformStamped(tf)

def print_tree():
  session = Session()
  node = session.query(FrameNode).\
                      filter(FrameNode.name == "world").\
                      first()

  msg(node.dump())

def print_nodes():
  session = Session()
  nodes = session.query(FrameNode).all()
  for node in nodes:
    print node

def test_lookup_frame(source_frame, target_frame):
  transform = lookup_transform(source_frame, target_frame)
  print source_frame, '->', target_frame, ':', transform

if __name__ == '__main__':

  ## SETUP
  #drop_all()
  #create_all()
  #create_tree()
  #print_tree()

  ## INSERT
  #test_ros_insert('world', 'floor0', [0,0,-1], [0,0,0,1])
  #test_ros_insert('table0', 'cup0', [0,0,0], [0,0,0,1])

  ## CHANGE REFERENCE
  #change_source("table0", "mug0")
  #change_source("table0", "mug1")

  ## REMOVE FRAMES
  #remove_frame('mug0')
  #remove_frame('table1', True)
  #remove_frame('room1', True, 'room0')

  ## LOOKUP FRAMES
  #test_lookup_frame("world", "mug1")
  #test_lookup_frame("floor1", "table0")

  ## GET ROS TRANSFORM
  #print rosGetTransform("world", "mug1")
  #print rosGetFrame("mug1")

  ## LOOKUP ROOTS
  #create_root_node('root')
  #print "all roots:", get_root_nodes()

  #print_nodes()

  msg('done')
