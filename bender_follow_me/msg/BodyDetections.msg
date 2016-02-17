
# frame_id y stamp
std_msgs/Header header

# detections
bender_follow_me/SingleBodyDetection[] detections

# Message source: NI, LEG, HOG, ....  
# this source should be defined somewhere as a package macro.
string source
