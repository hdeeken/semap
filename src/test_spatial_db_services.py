#!/usr/bin/env python

import roslib; roslib.load_manifest('spatial_db')

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
from spatial_db.msg import ObjectInstanceOverview as ROSObjectInstanceOverview

from spatial_db.srv import *
from tf.transformations import quaternion_matrix, random_quaternion, quaternion_from_matrix, euler_from_matrix, euler_matrix, quaternion_from_euler

import rospy

from visualization_msgs.msg import Marker, MarkerArray

def create_object_description():
  point2dmodel = Point2DModel()
  point2dmodel.type = '2dpoint'
  point2dmodel.geometry.x = -0.5
  point2dmodel.geometry.y = 0.0
  point2dmodel.geometry.z = 0.0

  pose2dmodel = Pose2DModel()
  pose2dmodel.type = '2dpose'
  pose2dmodel.pose = ROSPose2D()
  pose2dmodel.pose.x = 0.5
  pose2dmodel.pose.y = 1.5
  pose2dmodel.pose.theta = 1.5

  point0 = ROSPoint32(0, 0, 0)
  point1 = ROSPoint32(1, 0, 0)
  point2 = ROSPoint32(1, 1, 0)
  point3 = ROSPoint32(0, 1, 0)
  point4 = ROSPoint32(-1, 0, 0.5)
  point5 = ROSPoint32(-1, 1, 0.5)

  point6 = ROSPoint32(1, 1, 0)
  point7 = ROSPoint32(2, 1, 0)
  point8 = ROSPoint32(2, 2, 0)
  point9 = ROSPoint32(1, 2, 0)
  point10 = ROSPoint32(-2, 1, 0)
  point11 = ROSPoint32(-2, 2, 0)

  polygon0 = ROSPolygon()
  polygon0.points.append(point0)
  polygon0.points.append(point1)
  polygon0.points.append(point2)
  polygon0.points.append(point3)

  polygon1 = ROSPolygon()
  polygon1.points.append(point4)
  polygon1.points.append(point0)
  polygon1.points.append(point3)
  polygon1.points.append(point5)

  polygon2 = ROSPolygon()
  polygon2.points.append(point6)
  polygon2.points.append(point7)
  polygon2.points.append(point8)
  polygon2.points.append(point9)

  polygon3 = ROSPolygon()
  polygon3.points.append(point10)
  polygon3.points.append(point6)
  polygon3.points.append(point9)
  polygon3.points.append(point11)

  polygon2dmodel = Polygon2DModel()
  polygon2dmodel.type = '2dpolygon'
  polygon2dmodel.pose = ROSPose()
  polygon2dmodel.geometry = polygon0

  point3dmodel = Point3DModel()
  point3dmodel.type = '3dpoint'
  point3dmodel.geometry.x = 2.0
  point3dmodel.geometry.y = 2.0
  point3dmodel.geometry.z = 1.0

  pose3dmodel = Pose3DModel()
  pose3dmodel.type = '3dpose'
  pose3dmodel.pose = ROSPose()
  pose3dmodel.pose.position.x = -0.5
  pose3dmodel.pose.position.y = -0.5
  pose3dmodel.pose.position.z = 0.5

  rand_quat = random_quaternion()
  pose3dmodel.pose.orientation.x = rand_quat[0]
  pose3dmodel.pose.orientation.y = rand_quat[1]
  pose3dmodel.pose.orientation.z = rand_quat[2]
  pose3dmodel.pose.orientation.w = rand_quat[3]

  polygon3dmodel = Polygon3DModel()
  polygon3dmodel.type = '3dpolygon'
  polygon3dmodel.pose = ROSPose()
  polygon3dmodel.pose.position.x = 2.0
  polygon3dmodel.geometry = polygon1

  tri0 = ROSMeshTriangle()
  tri1 = ROSMeshTriangle()
  tri0.vertex_indices[0] = 0
  tri0.vertex_indices[1] = 1
  tri0.vertex_indices[2] = 2

  tri1.vertex_indices[0] = 0
  tri1.vertex_indices[1] = 2
  tri1.vertex_indices[2] = 3

  tpoint0 = ROSPoint( 0.0, 0.0, 0.0)
  tpoint1 = ROSPoint( 1.0, 0.0, 1.0)
  tpoint2 = ROSPoint( 1.0, 1.0, 0.0)
  tpoint3 = ROSPoint( 0.0, 1.0, 1.0)
  tpoint4 = ROSPoint(-1.0, 0.0, 0.0)
  tpoint5 = ROSPoint(-1.0, 1.0, 1.0)

  trianglemesh3dmodel = TriangleMesh3DModel()
  trianglemesh3dmodel.type = '3dtrianglemesh'
  trianglemesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  trianglemesh3dmodel.pose.position.x = 1.0
  trianglemesh3dmodel.pose.position.y = -2.0
  trianglemesh3dmodel.pose.position.z = 0.25
  #trianglemesh3dmodel.pose.orientation.x = rand_quat[0]
  #trianglemesh3dmodel.pose.orientation.y = rand_quat[1]
  #trianglemesh3dmodel.pose.orientation.z = rand_quat[2]
  #trianglemesh3dmodel.pose.orientation.w = rand_quat[3]
  trianglemesh3dmodel.geometry.vertices.append(tpoint0)
  trianglemesh3dmodel.geometry.vertices.append(tpoint1)
  trianglemesh3dmodel.geometry.vertices.append(tpoint2)
  trianglemesh3dmodel.geometry.vertices.append(tpoint3)
  trianglemesh3dmodel.geometry.triangles.append(tri0)
  trianglemesh3dmodel.geometry.triangles.append(tri1)

  polygonmesh3dmodel = PolygonMesh3DModel()
  polygonmesh3dmodel.type = '3dpolygonmesh'
  polygonmesh3dmodel.pose = ROSPose()
  rand_quat = random_quaternion()
  #polygonmesh3dmodel.pose.position.x = 0.5
  #polygonmesh3dmodel.pose.position.y = 1.0
  #polygonmesh3dmodel.pose.position.z = 0.0
  #polygonmesh3dmodel.pose.orientation.x = rand_quat[0]
  #polygonmesh3dmodel.pose.orientation.y = rand_quat[1]
  #polygonmesh3dmodel.pose.orientation.z = rand_quat[2]
  #polygonmesh3dmodel.pose.orientation.w = rand_quat[3]
  polygonmesh3dmodel.geometry.polygons.append(polygon2)
  polygonmesh3dmodel.geometry.polygons.append(polygon3)

  desc_ros = ROSObjectDescription()
  desc_ros.type = "test_description"
  desc_ros.point2d_models.append(point2dmodel)
  desc_ros.pose2d_models.append(pose2dmodel)
  desc_ros.polygon2d_models.append(polygon2dmodel)
  desc_ros.point3d_models.append(point3dmodel)
  desc_ros.pose3d_models.append(pose3dmodel)
  desc_ros.polygon3d_models.append(polygon3dmodel)
  desc_ros.trianglemesh3d_models.append(trianglemesh3dmodel)
  desc_ros.polygonmesh3d_models.append(polygonmesh3dmodel)

  return desc_ros

def create_object_instance():

  pose = ROSPoseStamped()
  pose.header.frame_id = "ref1"
  pose.pose.position.x = 0.0
  pose.pose.position.y = 1.0
  pose.pose.position.z = 0.0
  pose.pose.orientation.x = 1.0
  pose.pose.orientation.y = 2.0
  pose.pose.orientation.z = 9.0
  pose.pose.orientation.w = 1.0

  inst_ros = ROSObjectInstance()
  inst_ros.alias = "mr_objecto"
  inst_ros.pose = pose
  inst_ros.description = create_object_description()
  return inst_ros

#int32 id
#string type
#string default_model
#spatial_db/Point2DModel[] point2d_models
#spatial_db/Pose2DModel[] pose2d_models
#spatial_db/Polygon2DModel[] polygon2d_models
#spatial_db/Point3DModel[] point3d_models
#spatial_db/Pose3DModel[] pose3d_models
#spatial_db/Polygon3DModel[] polygon3d_models
#spatial_db/TriangleMesh3DModel[] trianglemesh3d_models
#spatial_db/PolygonMesh3DModel[] polygonmesh3d_models

def create_point_marker(point):
  marker = Marker()
  marker.ns = 'point'
  marker.header.frame_id = "base_link"
  marker.type = marker.SPHERE
  marker.action = marker.ADD
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.g = 1.0
  marker.color.a = 1.0
  marker.pose.position.x = point.x
  marker.pose.position.y = point.y
  marker.pose.position.z = point.z
  return marker

def create_polygon_marker(polygon, pose):
  marker = Marker()
  marker.ns = 'polygon'
  marker.header.frame_id = "base_link"
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.r = 1.0
  marker.color.a = 1.0
  marker.pose = pose
  marker.points = polygon.points
  marker.points.append(polygon.points[0])
  return marker

def create_mesh_marker(mesh, pose):
  marker = Marker()
  marker.ns = 'mesh'
  marker.header.frame_id = "base_link"
  marker.type = marker.TRIANGLE_LIST
  marker.action = marker.ADD
  marker.scale.x = 1.0
  marker.scale.y = 1.0
  marker.scale.z = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.color.a = 1.0
  marker.pose = pose

  for triangle in mesh.triangles:
    marker.points.append(mesh.vertices[triangle.vertex_indices[0]])
    marker.points.append(mesh.vertices[triangle.vertex_indices[1]])
    marker.points.append(mesh.vertices[triangle.vertex_indices[2]])

  return marker

def create_text_marker(text, pose):
  marker = Marker()
  marker.ns = 'text'
  marker.header.frame_id = "base_link"
  marker.type = marker.TEXT_VIEW_FACING
  marker.action = marker.ADD
  marker.scale.x = 0.15
  marker.scale.y = 0.15
  marker.scale.z = 0.15
  marker.color.b = 1.0
  marker.color.a = 1.0
  marker.pose = pose
  marker.text = text
  return marker

def create_pose_marker(pose):
  marker = Marker()
  marker.ns = 'pose'
  marker.header.frame_id = "base_link"
  marker.type = marker.ARROW
  marker.action = marker.ADD
  marker.scale.x = 0.25
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.b = 1.0
  marker.color.a = 1.0
  marker.pose = pose
  return marker

def create_object_description_marker(desc):
  array = MarkerArray()

  desc_type_marker = create_text_marker(desc.type, ROSPose())
  desc_type_marker.scale.x = 0.35
  desc_type_marker.scale.y = 0.35
  desc_type_marker.scale.z = 0.35
  desc_type_marker.color.r = 1.0
  desc_type_marker.color.g = 1.0
  desc_type_marker.color.b = 1.0
  desc_type_marker.color.a = 1.0
  desc_type_marker.pose.position.z = 0.75

  array.markers.append(desc_type_marker)

  for model in desc.point2d_models:
    array.markers.append(create_point_marker(model.geometry))
    pose = ROSPose()
    pose.position = model.geometry
    array.markers.append(create_text_marker(model.type, pose))
  for model in desc.pose2d_models:
    point = ROSPoint()
    point.x = model.pose.x
    point.y = model.pose.y
    point.z = 0.0
    pose = ROSPose()
    quat = quaternion_from_euler( 0, 0, model.pose.theta)
    pose.position = point
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    print pose

    array.markers.append(create_point_marker(point))
    array.markers.append(create_pose_marker(pose))
    array.markers.append(create_text_marker(model.type, pose))
  for model in desc.polygon2d_models:
    array.markers.append(create_polygon_marker(model.geometry, model.pose))
    array.markers.append(create_text_marker(model.type, model.pose))
  for model in desc.point3d_models:
    array.markers.append(create_point_marker(model.geometry))
    pose = ROSPose()
    pose.position = model.geometry
    array.markers.append(create_text_marker(model.type, pose))
  for model in desc.pose3d_models:
    array.markers.append(create_point_marker(model.pose.position))
    array.markers.append(create_pose_marker(model.pose))
    array.markers.append(create_text_marker(model.type, model.pose))
  for model in desc.polygon3d_models:
    array.markers.append(create_polygon_marker(model.geometry, model.pose))
    array.markers.append(create_text_marker(model.type, model.pose))
  for model in desc.trianglemesh3d_models:
     array.markers.append(create_mesh_marker(model.geometry, model.pose))
     array.markers.append(create_text_marker(model.type, model.pose))
  for model in desc.polygonmesh3d_models:
    array.markers.append(create_text_marker(model.type, model.pose))
    for polygon in model.geometry.polygons:
      poly = create_polygon_marker(polygon, model.pose)
      poly.color.r = 0.0
      poly.color.g = 1.0
      poly.color.b = 1.0
      poly.color.a = 1.0
      poly.ns = 'polymesh'
      array.markers.append(poly)

  id = 0
  for m in array.markers:
    m.id = id
    id += 1

  return array

def test_create_object_description_marker():
  rospy.init_node('publish_objects')

  publisher = rospy.Publisher('object_marker', MarkerArray)

  desc = create_object_description()
  array = create_object_description_marker(desc)
  #print array

  while not rospy.is_shutdown():
    publisher.publish(array)
    rospy.sleep(0.01)

def test_add_object_descriptions():
  try:
    rospy.wait_for_service('add_object_descriptions')
    request = AddObjectDescriptionsRequest()
    request.descriptions.append(create_object_description())
    add_object_descriptions_call = rospy.ServiceProxy('add_object_descriptions', AddObjectDescriptions)
    response = add_object_descriptions_call(request)
    print 'AddObjectDescriptions service call succeeded!'
    return True
  except rospy.ServiceException as e:
    return False, "AddObjectDescriptions service call failed: %s" % e

def test_add_object_instances():
  try:
    rospy.wait_for_service('add_object_instances')
    request = AddObjectInstancesRequest()
    request.objects.append(create_object_instance())
    add_object_instances_call = rospy.ServiceProxy('add_object_instances', AddObjectInstances)
    response = add_object_instances_call(request)
    print 'AddObjectInstances service call succeeded!'
    return True
  except rospy.ServiceException as e:
    return False, "AddObjectInstances service call failed: %s" % e

def test_get_all_object_instances():
  try:
    rospy.wait_for_service('get_all_object_instances')
    get_all_object_instances_call = rospy.ServiceProxy('get_all_object_instances', GetAllObjectInstances)
    #response = GetAllObjectInstancesResponse()
    response = get_all_object_instances_call()

    for object in response.objects:
      print 'name: ', object.alias, 'id: ', object.id, 'type: ', object.type

    print 'GetAllObjectInstances service call succeeded!'
    return True
  except rospy.ServiceException as e:
    return False, "GetAllObjectInstances service call failed: %s" % e

if __name__ == "__main__":

    #test_add_object_descriptions()
    #test_add_object_instances()
    #test_get_all_object_instances()
    test_create_object_description_marker()
    print 'done'
