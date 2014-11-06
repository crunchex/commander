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
  String openFilePath;
  String originalContents;
  
  bool shouldSave;
  
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
    
    resetSavePoint();
  }
  
  /// Sets up event handlers for the editor's menu buttons.
  void registerEditorEventHandlers() {
    cs.stream
        .where((m) => m.dest == 'EDITOR' && m.type == 'CLASS_ADD')
        .listen((m) => classAdd(m.body));
    
    cs.stream
        .where((m) => m.dest == 'EDITOR' && m.type == 'CLASS_REMOVE')
        .listen((m) => classRemove(m.body));
    
    // Editor receives command from Explorer to request file contents from the server.
    cs.stream
        .where((m) => m.dest == 'EDITOR' && m.type == 'OPEN_FILE')
        .listen((m) {
          ws.send('[[EDITOR_OPEN]]' + m.body);
        });
              
    // Editor receives the open file contents from the server.
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_FILE_TEXT')
        .listen((um) {
          var returnedData = um.body.split('[[CONTENTS]]');
          var newText = returnedData[1];
          if (noUnsavedChanges()) {
            openFilePath = returnedData[0];
            setEditorText(newText);
          } else {
            presentModal();
            modalSaveButton.onClick.listen((e) {
              saveText();
              openFilePath = returnedData[0];
              setEditorText(newText);
            });
            modalDiscardButton.onClick.listen((e) {
              openFilePath = returnedData[0];
              setEditorText(newText);
            });
          }
        });
    
    newButton.onClick.listen((e) {
      var newText = ROS_TALKER;
      if (noUnsavedChanges()) {
        openFilePath = absolutePathPrefix + 'untitled.cc';
        setEditorText(newText);
      } else {
        presentModal();
        modalSaveButton.onClick.listen((e) {
          saveText();
          openFilePath = absolutePathPrefix + 'untitled.cc';
          setEditorText(newText);
        });
        modalDiscardButton.onClick.listen((e) {
          openFilePath = absolutePathPrefix + 'untitled.cc';
          setEditorText(newText);
        });
      }

      e.preventDefault();
    });

    saveButton.onClick.listen((e) => saveText());
    
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
  void classAdd(String s) {
    editorDiv.classes.add(s);
  }
  
  void classRemove(String s) {
    editorDiv.classes.remove(s);
  }

  setEditorText(String newText) {
    aceEditor.setValue(newText, 1);
    resetSavePoint();
  }
  
  void presentModal() {
    DivElement modal = querySelector('#myModal');
    Modal m = new Modal(modal);
    m.show();
  }
  
  void saveText() {
    ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFilePath);
    resetSavePoint();
  }
  
  bool noUnsavedChanges() => aceEditor.value == originalContents;
  String resetSavePoint() => originalContents = aceEditor.value;
}