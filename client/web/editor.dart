part of client;

// Template for a new file.
// TODO: make this contain boilerplate ROS code
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

/// [UpDroidEditor] is a wrapper for an embedded Ace Editor. Sets styles
/// for the editor and an additional menu bar with some filesystem operations.
class UpDroidEditor {
  static const String EDITOR_FILE_TEXT = '[[EDITOR_FILE_TEXT]]';
  
  WebSocket ws;
  String absolutePathPrefix;

  AnchorElement saveButton;
  AnchorElement newButton;
  AnchorElement themeButton;
  ButtonElement modalSaveButton;
  ButtonElement modalDiscardButton;
  DivElement editorDiv;
  
  Editor aceEditor;
  String openFile;
  
  UpDroidEditor(WebSocket ws, int editorID) {
    this.ws = ws;
    editorDiv = querySelector('#editor-$editorID');
    
    saveButton = querySelector('#column-${editorID} .button-save');
    newButton = querySelector('#column-${editorID} .button-new');
    themeButton = querySelector('#column-${editorID} .button-editor-theme');
    modalSaveButton = querySelector('.modal-save');
    modalDiscardButton = querySelector('.modal-discard');
    
    setUpEditor();
    registerEditorEventHandlers();
  }

  /// Sets up the editor and styles.
  void setUpEditor() {
    implementation = ACE_PROXY_IMPLEMENTATION;
    
    aceEditor = edit(editorDiv);
    aceEditor
      ..session.mode = new Mode.named(Mode.PYTHON)
      ..fontSize = 14
      ..theme = new Theme.named(Theme.SOLARIZED_DARK);
  }
  
  void openTextHandler(String raw) {
    UpDroidMessage um = new UpDroidMessage(raw);
    aceEditor.setValue(um.body);
  }
  
  /// Sets up event handlers for the editor's menu buttons.
  void registerEditorEventHandlers() {
    ws.onMessage
        .where((value) => value.toString().startsWith(EDITOR_FILE_TEXT))
        .listen((value) => openTextHandler(value.toString()));
    
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
  
  // Helper methods for filesystem operations.
  void saveText() => ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFile);
}