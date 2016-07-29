# hlpr_kinesthetic_teaching

eef_publisher.py:
This code continually publishes the end effector pose (the '/base_link' to '/right_ee_link' transform) to the topic /eef_pose; this topic along with \joint_states is what the record demo action will bag.

record_keyframe_demo_action_server.py:
This is an action server that will record a single demo each time it is initiated (can be a keyrame or a trajectory demo).  Once the action is started, the action listens for messages on \record_demo_frame topic, messages have the following meaning: 
  msg = 0: start a trajectory demonstration ("like this")
  msg = 1: start a keyframe demonstration ("start here")
  msg = 2: record intermediate keyframe ("go here")
  msg = 3" last keyframe, or end of trajectory demo ("end here")
Each keyframe writes data to the bag file, bag is closed on last keyframe.  For trjaectory demonstrations, data is written to the bag file at 100hz

playback_keyframe_demo_AS.py:
This is an action server that will take a demo, stored as a bag file, and play it back using MoveIt to construct a plan and using hlpr_manipulation to execute the plan

playback_keyframe_demo_action_client.py, record_keyframe_demo_action_client.py: 
These are both little scripts if you want to quickly test out the above two action servers.  The record client has different examples of recording demonstrations.  The playback client tries to playback 'demo_data_0'.


