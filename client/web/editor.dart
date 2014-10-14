part of client;

void setUpEditor() {
  ace.implementation = ACE_PROXY_IMPLEMENTATION;
  
  ace.Editor editor = ace.edit(querySelector('#editor'));
  editor
      ..session.mode = new ace.Mode.named(ace.Mode.PYTHON)
      ..setValue(ROS_TALKER, -1);
}

const String ROS_TALKER =
r'''
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
''';