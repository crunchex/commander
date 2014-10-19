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

class UpDroidEditor {
  WebSocket ws;
  int id;

  AnchorElement saveButton;
  AnchorElement themeButton;
  
  Editor aceEditor;
  String openFile;
  
  UpDroidEditor(WebSocket ws, int id) {
    this.ws = ws;
    saveButton = querySelector('#button-save');
    themeButton = querySelector('#button-editor-theme');
    
    setUpEditor();
    registerEditorEventHandlers();
  }

  void setUpEditor() {
    implementation = ACE_PROXY_IMPLEMENTATION;
    
    aceEditor = edit(querySelector('#editor'));
    aceEditor
      ..session.mode = new Mode.named(Mode.PYTHON)
      ..fontSize = 14
      ..theme = new Theme.named(Theme.SOLARIZED_DARK)
      ..setValue(ROS_TALKER, -1);
  }
  
  void registerEditorEventHandlers() {
    saveButton.onClick.listen((e) {
      ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFile);
    });
    
    themeButton.onClick.listen((e) {
      if (aceEditor.theme.name == 'solarized_dark') {
        aceEditor.theme = new Theme.named(Theme.SOLARIZED_LIGHT);
      } else {
        aceEditor.theme = new Theme.named(Theme.SOLARIZED_DARK);
      }
      
      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });
  }
  
  void openText(String text) {
    aceEditor.setValue(text);
  }
}