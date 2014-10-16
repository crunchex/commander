part of client;

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

class Editor {
  AnchorElement themeButton;
  
  ace.Editor editor;
  
  Editor() {
    themeButton = querySelector('#button-editor-theme');
    
    setUpEditor();
    registerEditorEventHandlers();
  }

  void setUpEditor() {
    ace.implementation = ACE_PROXY_IMPLEMENTATION;
    
    editor = ace.edit(querySelector('#editor'));
    editor
        ..session.mode = new ace.Mode.named(ace.Mode.PYTHON)
        ..fontSize = 14
        ..theme = new ace.Theme.named(ace.Theme.SOLARIZED_DARK)
        ..setValue(ROS_TALKER, -1);
  }
  
  void registerEditorEventHandlers() {
    themeButton.onClick.listen((e) {
      if (editor.theme.name == 'solarized_dark') {
        editor.theme = new ace.Theme.named(ace.Theme.SOLARIZED_LIGHT);
      } else {
        editor.theme = new ace.Theme.named(ace.Theme.SOLARIZED_DARK);
      }
      
      // Stops the button from sending the page to the top (href=#)
      e.preventDefault();
    });
  }
}