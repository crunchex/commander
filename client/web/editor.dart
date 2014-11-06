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
  WebSocket ws;
  StreamController<CommanderMessage> cs;
  String absolutePathPrefix;

  AnchorElement saveButton;
  AnchorElement newButton;
  AnchorElement themeButton;
  ButtonElement modalSaveButton;
  ButtonElement modalDiscardButton;
  DivElement editorDiv;
  
  Editor aceEditor;
  String openFile;
  String originalContents;
  
  UpDroidEditor(WebSocket ws, StreamController<CommanderMessage> cs, String path, int editorID) {
    this.ws = ws;
    this.cs = cs;
    absolutePathPrefix = path;
    
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
    
    originalContents = aceEditor.value;
  }
  
  /// Process messages that Console has picked up according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CLASS_ADD':
        classAdd(m.body);
        break;
        
      case 'CLASS_REMOVE':
        classRemove(m.body);
        break;
        
      case 'OPEN_FILE':
        openFile = m.body;
        ws.send('[[EDITOR_OPEN]]' + openFile);
        break;
        
      default:
        print('Console error: unrecognized message type.');
    }
  }
  
  void classAdd(String s) {
    editorDiv.classes.add(s);
  }
  
  void classRemove(String s) {
    editorDiv.classes.remove(s);
  }
  
  void openTextHandler(String data) {
    originalContents = data;
    aceEditor.setValue(data);
  }
  
  /// Sets up event handlers for the editor's menu buttons.
  void registerEditorEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_FILE_TEXT')
        .listen((um) => openTextHandler(um.body));
    
    cs.stream
        .where((m) => m.dest == 'EDITOR')
        .listen((m) => processMessage(m));
    
    saveButton.onClick.listen((e) => saveText());
    
    newButton.onClick.listen((e) {
      if (aceEditor.value != originalContents) {
        DivElement modal = querySelector('#myModal');
        Modal m = new Modal(modal);
        m.show();
      } else {
        newFile();
      }

      e.preventDefault();
    });
    
    modalSaveButton.onClick.listen((e) {
      saveText();
      newFile();
    });
    
    modalDiscardButton.onClick.listen((e) => newFile());
    
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
  void saveText() {
    ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFile);
    originalContents = aceEditor.value;
  }
  
  void newFile() {
    openFile = absolutePathPrefix + 'untitled.cc';
    aceEditor.setValue(ROS_TALKER, 1);
  }
}