#!/usr/bin/env python
import roslib; roslib.load_manifest('spatial_db')

from sets import Set
from db_environment import Base
from db_environment import Session

from sqlalchemy import Column, ForeignKey, Integer, String, create_engine
from sqlalchemy.orm import Session, relationship, backref,\
                                joinedload_all
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm.collections import attribute_mapped_collection

from sqlalchemy import Column, Integer, String, Float, ForeignKey
from sqlalchemy.orm import relationship, backref

from geoalchemy2.types import Geometry
from geoalchemy2.elements import WKTElement, WKBElement, RasterElement, CompositeElement
from geoalchemy2.functions import ST_Distance, ST_AsText
from geoalchemy2.compat import buffer, bytes
from postgis_functions import *

from sets import Set
from db_environment import Base
from db_environment import Session
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Point32 as ROSPoint32
from geometry_msgs.msg import Pose2D as ROSPose2D
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Polygon as ROSPolygon
from shape_msgs.msg import Mesh as ROSMesh
from shape_msgs.msg import MeshTriangle as ROSMeshTriangle
from spatial_db.msg import PolygonMesh as ROSPolygonMesh
from spatial_db.msg import Point2DModel, Point3DModel, Pose2DModel, Pose3DModel, Polygon2DModel, Polygon3DModel, TriangleMesh3DModel, PolygonMesh3DModel
from spatial_db.msg import ObjectDescription as ROSObjectDescription
from spatial_db.msg import ObjectInstance as ROSObjectInstance

from numpy import radians
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix

class TreeNode(Base):
    __tablename__ = 'tree'
    id = Column(Integer, primary_key=True)
    parent_id = Column(Integer, ForeignKey(id))
    name = Column(String(50), nullable=False)
    pose = Column(Integer, nullable=False)

    children = relationship("TreeNode",

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

    def __init__(self, name, pose, parent=None):
        self.name = name
        self.parent = parent
        self.pose = pose

    def __repr__(self):
        return "TreeNode(name=%r, id=%r, parent_id=%r, pose=%r)" % (
                    self.name,
                    self.id,
                    self.parent_id,
                    self.pose
                )

    def dump(self, _indent=0):
        return "   " * _indent + repr(self) + \
                    "\n" + \
                    "".join([
                        c.dump(_indent + 1)
                        for c in self.children.values()]
                    )
