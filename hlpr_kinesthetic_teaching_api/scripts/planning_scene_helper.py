#!/usr/bin/env python
"""
Helper functions for MoveIt planning scene interface - add and remove
collision objects. Adapted from code by Andrew Sharp
"""

__author__ = "Andrew Sharp, Adam Allevato"
__license__ = "BSD"
__copyright__ = """Copyright The University of Texas at Austin, 2014-2018.
                All rights reserved. This software and documentation
                constitute an unpublished work and contain valuable trade
                secrets and proprietary information belonging to the
                University. None of the foregoing material may be copied or
                duplicated or disclosed without the express, written
                permission of the University. THE UNIVERSITY EXPRESSLY
                DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND
                DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY
                AND/OR FITNESS FOR A PARTICULAR PURPOSE, AND WARRANTIES OF
                PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM
                COURSE OF DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER
                EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF THE SOFTWARE OR
                DOCUMENTATION. Under no circumstances shall the University be
                liable for incidental, special, indirect, direct or
                consequential damages or loss of profits, interruption of
                business, or related expenses which may arise from use of
                software or documentation, including but not limited to those
                resulting from defects in software and/or documentation, or
                loss or inaccuracy of data of any kind."""

import os
from rospkg import RosPack

from tf import transformations
from pyassimp.errors import AssimpError
from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from rospy import loginfo, logerr, logwarn, logdebug, get_param, Publisher, \
    Time, ROSException, ROSSerializationException, sleep, get_rostime
from shape_msgs.msg import SolidPrimitive



class PlanningSceneHelper:
    def __init__(self, package=None, mesh_folder_path=None):
        # interface to the planning and collision scenes
        self.psi = PlanningSceneInterface()
        self.scene_pub = Publisher("/collision_object", CollisionObject,
                                   queue_size=1, latch=True)
        self.vis_pub = Publisher("/collision_scene", MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        sleep(1.0)

        if package is not None and mesh_folder_path is not None:
            # Build folder path for use in load
            rospack = RosPack()
            self.package = package
            self.mesh_folder_path = mesh_folder_path
            self.package_path = rospack.get_path(package)
            self.folder_path = os.path.join(self.package_path, mesh_folder_path)\
            + os.sep
        else:
            logwarn("Missing package or mesh folder path; no meshes can be added")

        # Create dict of objects, for removing markers later
        self.objects = {}
        self.next_id = 1

    # Loads the selected mesh file into collision scene at the desired location.
    def add_mesh(self, object_id, frame, filename, visual=None, pose=None,
                 position=None, orientation=None, rpy=None, color=None):
        if self.package is None:
            logwarn("No package was provided; no meshes can be loaded.")
        
        if visual is None:
            visual = filename

        filename = self.folder_path + filename
        stamped_pose = self.make_stamped_pose(frame=frame,
                                              pose=pose,
                                              position=position,
                                              orientation=orientation,
                                              rpy=rpy)
        try:
            self.psi.add_mesh(object_id, stamped_pose, filename)
            loginfo("Loaded " + object_id + " from" + filename +
                     " as collision object.")

            marker = self.make_marker_stub(stamped_pose, color=color)
            marker.type = Marker.MESH_RESOURCE
            # marker.mesh_resource = "file:/" + self.folder_path + visual
            marker.mesh_resource = "package://" + self.package + os.sep + self.mesh_folder_path + os.sep + visual

            self.marker_array = MarkerArray()
            self.marker_array.markers.append(marker)
            self.vis_pub.publish(self.marker_array)
            self.objects[object_id] = marker

            loginfo("Loaded " + object_id + " from " + marker.mesh_resource +
                     " as marker.")

        except ROSException as e:
            loginfo("Problem loading " + object_id +
                    " collision object: " + str(e))
        except AssimpError as ae:
            loginfo("Problem loading " + object_id +
                    " collision object: " + str(ae))

    def add_cylinder(self, object_id, frame, size, pose=None,
                     position=None, orientation=None, rpy=None, color=None):
        try:
            stamped_pose = self.make_stamped_pose(frame=frame,
                                                  pose=pose,
                                                  position=position,
                                                  orientation=orientation,
                                                  rpy=rpy)

            # Cylinders are special - they are not supported directly by
            # moveit_commander. It must be published manually to collision scene
            cyl = CollisionObject()
            cyl.operation = CollisionObject.ADD
            cyl.id = object_id
            cyl.header = stamped_pose.header

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [size[0], size[1]]
            cyl.primitives = [prim]
            cyl.primitive_poses = [stamped_pose.pose]
            self.scene_pub.publish(cyl)

            marker = self.make_marker_stub(stamped_pose, [size[1]*2, size[2]*2, size[0]], color=color)
            marker.type = Marker.CYLINDER
            self.publish_marker(object_id, marker)

            sleep(0.5)
            logdebug("Loaded " + object_id + " as cylindrical collision object.")
        except ROSException as e:
            loginfo("Problem loading " + object_id +
                    " collision object: " + str(e))

    def add_sphere(self, object_id, frame, size, pose=None, position=None,
                   orientation=None, rpy=None, color=None):
        try:
            stamped_pose = self.make_stamped_pose(frame=frame,
                                                  pose=pose,
                                                  position=position,
                                                  orientation=orientation,
                                                  rpy=rpy)

            self.psi.add_sphere(object_id, stamped_pose, size[0])
            loginfo("got past adding collision scene object");

            marker = self.make_marker_stub(stamped_pose, [size[0]*2, size[1]*2, size[2]*2], color=color)
            marker.type = Marker.SPHERE
            self.publish_marker(object_id, marker)
            sleep(0.5)

            loginfo("Loaded " + object_id + " as collision object.")
        except ROSException as e:
            loginfo("Problem loading " + object_id +
                    " collision object: " + str(e))

    def add_box(self, object_id, frame, size, pose=None, position=None,
                orientation=None, rpy=None, color=None):
        try:
            stamped_pose = self.make_stamped_pose(frame=frame,
                                                  pose=pose,
                                                  position=position,
                                                  orientation=orientation,
                                                  rpy=rpy)

            self.psi.add_box(object_id, stamped_pose, size)

            marker = self.make_marker_stub(stamped_pose, size, color=color)
            marker.type = Marker.CUBE
            self.publish_marker(object_id, marker)
            sleep(0.5)

            loginfo("Loaded " + object_id + " as collision object.")
        except ROSException as e:
            loginfo("Problem loading " + object_id +
                    " collision object: " + str(e))

    def attach_box(self, link, object_id, frame, size, attach_to_link,
                   pose=None, position=None, orientation=None, rpy=None):
        """TODO: color is not yet supported, since it's not internally
        supported by psi.attach_box. Basically duplicate this method
        but with color support."""
        try:
            stamped_pose = self.make_stamped_pose(frame=frame,
                                                  pose=pose,
                                                  position=position,
                                                  orientation=orientation,
                                                  rpy=rpy),
            self.psi.attach_box(link, object_id, stamped_pose, size,
                                attach_to_link)
            sleep(0.5)

            loginfo("Attached " + object_id + " as collision object.")
        except ROSException as e:
            loginfo("Problem attaching " + object_id +
                    " collision object: " + str(e))

    @staticmethod
    def make_stamped_pose(frame, pose=None, position=None, orientation=None,
                          rpy=None):
        if orientation is not None and rpy is not None:
            logwarn("Collision object has both orientation (quaternion) and "
                    "Rotation (rpy) defined! Defaulting to quaternion "
                    "representation")

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = frame
        stamped_pose.header.stamp = Time.now()

        # use a pose if one is provided, otherwise, make your own from the
        # position and orientation.
        if pose is not None:
            stamped_pose.pose = pose
        else:
            stamped_pose.pose.position.x = position[0]
            stamped_pose.pose.position.y = position[1]
            stamped_pose.pose.position.z = position[2]
            # for orientation, allow either quaternion or RPY specification
            if orientation is not None:
                stamped_pose.pose.orientation.x = orientation[0]
                stamped_pose.pose.orientation.y = orientation[1]
                stamped_pose.pose.orientation.z = orientation[2]
                stamped_pose.pose.orientation.w = orientation[3]
            elif rpy is not None:
                quaternion = transformations.quaternion_from_euler(rpy[0],
                                                                   rpy[1],
                                                                   rpy[2])
                stamped_pose.pose.orientation.x = quaternion[0]
                stamped_pose.pose.orientation.y = quaternion[1]
                stamped_pose.pose.orientation.z = quaternion[2]
                stamped_pose.pose.orientation.w = quaternion[3]
            else:
                stamped_pose.pose.orientation.w = 1  # make basic quaternion
        return stamped_pose

    # Fill a ROS Marker message with the provided data
    def make_marker_stub(self, stamped_pose, size=None, color=None):
        if color is None:
            color = (0.5, 0.5, 0.5, 1.0)
        if size is None:
            size = (1, 1, 1)

        marker = Marker()
        marker.header = stamped_pose.header
        # marker.ns = "collision_scene"
        marker.id = self.next_id
        marker.lifetime = Time(0)  # forever
        marker.action = Marker.ADD
        marker.pose = stamped_pose.pose
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        if len(color) == 4:
            alpha = color[3]
        else:
            alpha = 1.0
        marker.color.a = alpha
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        self.next_id += 1
        return marker

    # publish a single marker
    def publish_marker(self, object_id, marker):
        loginfo("Publishing marker for object {}".format(object_id))
        self.marker_array = MarkerArray()
        self.marker_array.markers.append(marker)
        self.vis_pub.publish(self.marker_array)
        self.objects[object_id] = marker

    # Remove the provided object from the world.
    def remove(self, object_id):
        if object_id not in self.objects:
            logwarn("PlanningSceneHelper was not used to create object with id"
                    + object_id + ". Attempting to remove it anyway.")
        try:
            # first remove the actual collision object
            self.psi.remove_world_object(object_id)

            if object_id in self.objects:
                # then remove the marker associated with it
                marker = self.objects[object_id]
                marker.action = Marker.DELETE
                self.publish_marker(object_id, marker)
                self.objects.pop(object_id)
            logdebug("Marker for collision object " + object_id + " removed.")
        except ROSException as e:
            loginfo("Problem removing " + object_id + " from collision scene:"
                    + str(e))
            return
        except KeyError as e:
            loginfo("Problem removing " + object_id + " from collision scene:"
                    + str(e))
            return
            
        loginfo("Model " + object_id + " removed from collision scene.")

    # Remove the provided attached collision object.
    def remove_attached(self, link, object_id):
        try:
            self.psi.remove_attached_object(link=link, name=object_id)
            loginfo("Attached object '" + object_id +
                    "' removed from collision scene.")
        except:
            loginfo("Problem attached object '" + object_id +
                    "' removing from collision scene.")
