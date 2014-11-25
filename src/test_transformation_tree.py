#!/usr/bin/env python

#import roslib; roslib.load_manifest('spatial_db')

from db_environment import Base
from db_environment import Session

from db_transformation_tree_model import *
from sqlalchemy.orm import aliased

from sqlalchemy import Column, ForeignKey, Integer, String, func #, create_engine
from sqlalchemy.orm import relationship, backref,\
                                joinedload_all
#from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm.collections import attribute_mapped_collection

session = Session()

def msg(msg, *args):
  msg = msg % args
  print("\n\n\n" + "-" * len(msg.split("\n")[0]))
  print(msg)
  print("-" * len(msg.split("\n")[0]))

def original_example():
    msg("Creating Tree Table:")

    node = TreeNode('rootnode')
    TreeNode('node1', parent=node)
    TreeNode('node3', parent=node)

    node2 = TreeNode('node2')
    TreeNode('subnode1', parent=node2)
    node.children['node2'] = node2
    TreeNode('subnode2', parent=node.children['node2'])

    msg("Created new tree structure:\n%s", node.dump())

    msg("flush + commit:")

    session.add(node)
    session.commit()

    msg("Tree After Save:\n %s", node.dump())

    TreeNode('node4', parent=node)
    TreeNode('subnode3', parent=node.children['node4'])
    TreeNode('subnode4', parent=node.children['node4'])
    TreeNode('subsubnode1', parent=node.children['node4'].children['subnode3'])

    msg("Tree after adding some subnodes:\n %s", node.dump())

    # remove node1 from the parent, which will trigger a delete
    # via the delete-orphan cascade.
    del node.children['node1']

    msg("Removed node1.  flush + commit:")
    session.commit()

    msg("Tree after save:\n %s", node.dump())

    msg("Emptying out the session entirely, "
        "selecting tree on root, using eager loading to join four levels deep.")
    session.expunge_all()
    node = session.query(TreeNode).\
                        options(joinedload_all("children", "children",
                                                "children", "children")).\
                        filter(TreeNode.name == "rootnode").\
                        first()

    msg("Full Tree:\n%s", node.dump())

    msg("Marking root node as deleted, flush + commit:")

    session.delete(node)
    session.commit()

def create_tree():
  msg("Creating Tree Table:")

  node = TreeNode('rootnode', 0)
  TreeNode('node1', parent=node, pose=1)

  node2 = TreeNode('node2', pose=2)
  TreeNode('subnode1', parent=node2, pose = 0)
  node.children['node2'] = node2
  subnode2 = TreeNode('subnode2', parent=node.children['node2'], pose=1)
  subsubnode = TreeNode('subsubnode', parent=node2.children['subnode2'], pose=1)
  subsubsubnode = TreeNode('subsubsubnode', parent=node2.children['subnode2'].children['subsubnode'], pose=5)
  TreeNode('subsubsubsubnode', parent=subsubsubnode, pose=2)

  session.add(node)
  session.commit()

  msg("Tree After Save:\n %s", node.dump())

def print_tree():
  node = session.query(TreeNode).\
                      filter(TreeNode.name == "rootnode").\
                      first()

                      #options(joinedload_all("children", "children",
                       #                      "children", "children", "children")).\

  msg(node.dump())

def test():
  included_parts = session.query(
                TreeNode.id,
                TreeNode.name,
                TreeNode.pose).\
                    filter(TreeNode.name=="rootnode").\
                    cte(name="included_parts", recursive=True)

  incl_alias = aliased(included_parts, name="pr")
  parts_alias = aliased(TreeNode, name="p")
  included_parts = included_parts.union_all(
    session.query(
        parts_alias.id,
        parts_alias.name,
        parts_alias.pose).\
            filter(parts_alias.id==incl_alias.c.id)
    )

  q = session.query(
        included_parts.c.id,
        func.sum(included_parts.c.pose).
            label('total_quantity')
    ).\
    group_by(included_parts.c.id).scalar()

  print q

if __name__ == '__main__':

  #create_tree()
  #print_tree()
  test()
  msg('done')









