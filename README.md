# hlpr_kinesthetic_teaching

Beware, this code is pretty ripe. But here is the intended functionality.

eef_publisher.py:
This code continually publishes the end effector pose (the '/base_link' to '/right_ee_link' transform) to the topic /eef_pose

record_keyframe_demo_action_server.py:
This is an action server that will record a single keyframe demo each time it is initiated. Once the action is started, keyframes are triggered with the message \record_demo_frame a bag is created with the name passed in, each keyframe writes data to bag, bag closed on last kf

playback_keyframe_demo_AS.py:
This is an action server that will take a keyframe demo, stored as a bag file, and play it back.  

playback_keyframe_demo_action_client.py, record_keyframe_demo_action_client.py: 
These are both little scripts if you want to quickly test out the above two action servers.  The record client waits for keyboard input between each of 3 keyframes and prints the contents of the resulting bag file.  The playback client tries to playback 'demo_data_0'.


