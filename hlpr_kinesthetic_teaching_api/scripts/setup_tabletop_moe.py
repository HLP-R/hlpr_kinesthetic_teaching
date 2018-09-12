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
from planning_scene_helper import PlanningSceneHelper

if __name__=="__main__":
    rospy.init_node("planning_scene_setup")
    p = PlanningSceneHelper()

    p.remove("table")

    p.remove("torso")
    p.remove("head")
    p.remove("base")
    
    p.remove("left")
    p.remove("right")
    p.remove("back")
    
    p.add_box("base", "base_link", size=(.9,.6,.5), position=(0.,0.,.25), color=(0,0,1.,0.3))
    p.add_box("torso", "base_link", size=(.25,.4,.5), position=(0.,0.,.5+.25), color=(0,0,1.,0.3))
    p.add_sphere("head", "base_link", size=(.2,.2,.2), position=(0.,0.,1+.1), color=(0,0,1.,0.3))
    
    p.add_box("table", "base_link", size=(1.5,1.5,.62), position=(1.,0.,.31), color=(0,0,1.,1.))

    p.add_box("left", "base_link", size=(2,.3,3.0), position=(1.,-2,1.5), color=(0,0,1.,0.4))
    p.add_box("right", "base_link", size=(2,.3,3.0), position=(1.,2,1.5), color=(0,0,1.,0.4))
    p.add_box("back", "base_link", size=(.3,2.,3.0), position=(-.15,0.,1.5), color=(0,0,1.,0.4))
    
    
