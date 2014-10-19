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
  String absolutePathPrefix;

  AnchorElement saveButton;
  AnchorElement newButton;
  AnchorElement themeButton;
  ButtonElement modalSaveButton;
  ButtonElement modalDiscardButton;
  
  Editor aceEditor;
  String openFile;
  
  UpDroidEditor(WebSocket ws, int id) {
    this.ws = ws;
    saveButton = querySelector('#button-save');
    newButton = querySelector('#button-new');
    themeButton = querySelector('#button-editor-theme');
    modalSaveButton = querySelector('#modal-save');
    modalDiscardButton = querySelector('#modal-discard');
    
    setUpEditor();
    registerEditorEventHandlers();
  }

  void setUpEditor() {
    implementation = ACE_PROXY_IMPLEMENTATION;
    
    aceEditor = edit(querySelector('#editor'));
    aceEditor
      ..session.mode = new Mode.named(Mode.PYTHON)
      ..fontSize = 14
      ..theme = new Theme.named(Theme.SOLARIZED_DARK);
  }
  
  void registerEditorEventHandlers() {
    saveButton.onClick.listen((e) => saveText());
    
    newButton.onClick.listen((e) {
      ParagraphElement p = querySelector('#modal-save-text');
      p.appendText('You have made more changes since the last save. Save these changes?');
    });
    
    modalSaveButton.onClick.listen((e) {
      saveText();
      openFile = absolutePathPrefix + 'untitled.cc';
      aceEditor.setValue(ROS_TALKER, 1);
    });
    
    modalDiscardButton.onClick.listen((e) {
      openFile = absolutePathPrefix + 'untitled.cc';
      aceEditor.setValue(ROS_TALKER, 1);
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
  
  String openText(String text) => aceEditor.setValue(text);
  void saveText() => ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFile);
}