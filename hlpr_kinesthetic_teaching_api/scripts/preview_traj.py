#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import rosbag
import sys
import os
from shutil import copyfile

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from kinova_msgs.srv import Start, Stop
from hlpr_manipulation_utils.manipulator import Gripper
import tf

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
from matplotlib import collections  as mc


class TrajectoryDisplay:
    def __init__(self, plans, planner):
        self.plans = plans
        self.planner = planner
        self.fig = plt.figure(facecolor="w")

        self.n_joints = len(self.get_plan_poses(0)[0])
        
        self.joint_axes = []
        for i in range(self.n_joints):
            self.joint_axes.append(self.fig.add_subplot(self.n_joints, 3, i*3+1))
        
        self.eef_xyz = self.fig.add_subplot(3,3,8)
        self.eef_rpy = self.fig.add_subplot(3,3,9)

        self.main_plot = plt.subplot2grid((3,3),(0,1),colspan=2,rowspan=2, projection='3d')
        



    def update_plots(self):
        for axis in self.joint_axes + [self.eef_xyz, self.eef_rpy, self.main_plot]:
            axis.clear()
        
        cmap = matplotlib.cm.get_cmap('viridis')
        
        joint_kfs = []
        time_kfs = []
        colors = []
        color_kfs = []
        eef_kfs = []
        for i in range(len(self.plans)):
            joints = self.get_plan_poses(i)
            times = self.get_plan_times(i)
            color = cmap(i*(1.0/(len(self.plans)-1)))
            eefs = self.get_plan_eef(i)
            self.eef_xyz.plot(times, map(lambda p: p[0], eefs), color=color)
            self.eef_xyz.plot(times, map(lambda p: p[1], eefs), color=color)
            self.eef_xyz.plot(times, map(lambda p: p[2], eefs), color=color)
            self.main_plot.plot(xs=map(lambda p: p[0], eefs),ys= map(lambda p: p[1], eefs),zs=map(lambda p: p[2], eefs),color=color, linewidth=3)
        
            self.eef_rpy.plot(times, map(lambda p: p[3], eefs), color=color)
            self.eef_rpy.plot(times, map(lambda p: p[4], eefs), color=color)
            self.eef_rpy.plot(times, map(lambda p: p[5], eefs), color=color)

            for i in range(self.n_joints):
                self.joint_axes[i].plot(times,map(lambda p: p[i], joints),color=color)
        
            eef_kfs.append(eefs[-1])
            joint_kfs.append(joints[-1])
            time_kfs.append(times[-1])
            color_kfs.append(color)
            
        joint_locs = None
        joint_kf_idxs = []
        joint_colors = []
        for i in range(len(self.plans)):
            new_joints = self.get_plan_joints_cartesian(i,7)
            if joint_locs is None:
                joint_locs = new_joints
            else:
                for j in range(len(new_joints)):
                    joint_locs[j] = joint_locs[j]+new_joints[j]
            joint_kf_idxs.append(len(joint_locs[0])-1)
            color = list(cmap(i*(1.0/(len(self.plans)-1))))
            color[3] = 0.5
            color = tuple(color)
            color = (0,0,0,0.5)
            joint_colors+=[color]*len(new_joints[0])
        
        
        for i in range(self.n_joints):
            self.joint_axes[i].scatter(time_kfs, map(lambda p: p[i], joint_kfs))


        self.eef_xyz.scatter(time_kfs, map(lambda p:p[0],eef_kfs), color=color_kfs)
        self.eef_xyz.scatter(time_kfs, map(lambda p:p[1],eef_kfs), color=color_kfs)
        self.eef_xyz.scatter(time_kfs, map(lambda p:p[2],eef_kfs), color=color_kfs)
        self.eef_rpy.scatter(time_kfs, map(lambda p:p[3],eef_kfs), color=color_kfs)
        self.eef_rpy.scatter(time_kfs, map(lambda p:p[4],eef_kfs), color=color_kfs)
        self.eef_rpy.scatter(time_kfs, map(lambda p:p[5],eef_kfs), color=color_kfs)

        self.main_plot.scatter(xs=map(lambda p: p[0], eef_kfs),ys= map(lambda p: p[1], eef_kfs),zs=map(lambda p: p[2], eef_kfs))
        
        lines = []
        for i in range(len(joint_locs[0])):
            xs = []
            ys = []
            zs = []
            for j in range(len(joint_locs)):
                xs.append(joint_locs[j][i][0])
                ys.append(joint_locs[j][i][1])
                zs.append(joint_locs[j][i][2])
            self.main_plot.plot(xs,ys,zs, color=joint_colors[i], linewidth = 1.5)
            #self.main_plot.scatter(xs,ys,zs,  color=(0,0,0,1), s = 2)

        '''for a in range(3):
            eef_rot = self.get_eef_vect(0,a)


            eef_x = map(lambda p: p[0], eefs)
            eef_y = map(lambda p: p[1], eefs)
            eef_z = map(lambda p: p[2], eefs)

            eef_u = map(lambda p: p[0], eef_rot)
            eef_v = map(lambda p: p[1], eef_rot)
            eef_w = map(lambda p: p[2], eef_rot)

            color = [0,0,0,1]
            color[a]=1
            
            self.main_plot.quiver(eef_x,eef_y,eef_z,eef_u,eef_v,eef_w, length=0.1, pivot="tail", colors=[color]*len(eef_x))'''



        for axis in self.joint_axes + [self.eef_xyz, self.eef_rpy, self.main_plot]:
            axis.autoscale()

        plt.show()


    def get_eef_vect(self, plan_num, axis=0):
        traj = self.plans[plan_num].joint_trajectory
        eefs = map(lambda t: self.planner.get_FK(state = self.planner.state_from_joints(dict(zip(traj.joint_names,t.positions))))[0].pose, traj.points)

        
        axis_vect = [0,0,0]
        axis_vect[axis]=1
        axis_vect = tf.transformations.unit_vector(axis_vect)

        def rotate_axis(orientation):
            rot = tf.transformations.quaternion_matrix([orientation.x,orientation.y,orientation.z,orientation.w])
            vect = np.append(axis_vect,0.0)
            return list(np.ndarray.flatten((rot*vect)[:3,axis]))
        
        directions = map(lambda e: rotate_axis(e.orientation), eefs)
        return directions

        
    def get_plan_poses(self, plan_num):
        return map(lambda t: t.positions, self.plans[plan_num].joint_trajectory.points)

    def get_plan_times(self, plan_num):
        if plan_num > 0:
            time_adj = 0
            for i in range(0,plan_num):
                time_adj+=self.plans[i].joint_trajectory.points[-1].time_from_start.to_sec()+0.5
        else:
            time_adj = 0
        return map(lambda t: t.time_from_start.to_sec()+time_adj, self.plans[plan_num].joint_trajectory.points)

    def get_plan_eef(self, plan_num):
        traj = self.plans[plan_num].joint_trajectory
        eefs = map(lambda t: self.planner.get_FK(state = self.planner.state_from_joints(dict(zip(traj.joint_names,t.positions))))[0].pose, traj.points)
        
        def xyzrpy(pose):
            quaternion = [pose.orientation.x,
                          pose.orientation.y,
                          pose.orientation.z,
                          pose.orientation.w]
            rpy = list(tf.transformations.euler_from_quaternion(quaternion))
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            return xyz+rpy

        return map(lambda p: xyzrpy(p), eefs)
        
    def get_plan_joints_cartesian(self, plan_num, skip=1):
        traj = self.plans[plan_num].joint_trajectory
        
        paths = []

        points = [traj.points[i] for i in range(len(traj.points)) if i==0 or i==len(traj.points)-1 or i%skip==0]
        
        states = map(lambda t: self.planner.state_from_joints(dict(zip(traj.joint_names,t.positions))), points)

        
        def xyz(pose):
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            return xyz


        arm_links = ["j2s7s300_link_base"]+["j2s7s300_link_{}".format(i) for i in [1,2,3,4,5,6,7]]+["j2s7s300_ee_link"]
        
        for joint_name in arm_links:
            poses = map(lambda st: self.planner.get_FK(state = st, target=joint_name)[0].pose, states)
            paths.append(map(lambda p: xyz(p),poses))

        return paths

if __name__=="__main__":
    rospy.init_node("preview_trajectory")
    
    a = ArmMoveIt(planning_frame="base_link")
    a.group[0].set_end_effector_link("j2s7s300_ee_link")


    n_points = 3
    eefs = []
    joints = []

    start = a.get_IK(a.get_random_reachable().pose)
    
    for i in range(n_points):
        eef = a.get_random_reachable()
        eefs.append(eef)
        joints.append(a.get_IK(eef.pose))

    plans = a.plan_joint_waypoints(joints, starting_config = a.state_from_joints(start))
    if plans is None:
        print "no plan found :("
    else:
        t = TrajectoryDisplay(plans, a)
        t.update_plots()
    
