part of updroid_client;

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

  DivElement editorDiv;
  AnchorElement saveButton;
  AnchorElement newButton;
  AnchorElement themeButton;
  ButtonElement modalSaveButton;
  ButtonElement modalDiscardButton;
  
  Editor aceEditor;
  String openFilePath;
  String originalContents;

  UpDroidEditor(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    editorDiv = querySelector('#editor');
    
    saveButton = querySelector('#column-1 .button-save');
    newButton = querySelector('#column-1 .button-new');
    themeButton = querySelector('#column-1 .button-editor-theme');
    modalSaveButton = querySelector('.modal-save');
    modalDiscardButton = querySelector('.modal-discard');
    
    setUpEditor();
    registerEditorEventHandlers();
    
    cs.add(new CommanderMessage('CLIENT', 'EDITOR_READY'));
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
        .listen((m) => editorDiv.classes.add(m.body));
    
    cs.stream
        .where((m) => m.dest == 'EDITOR' && m.type == 'CLASS_REMOVE')
        .listen((m) => editorDiv.classes.remove(m.body));
    
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
          var newPath = returnedData[0];
          var newText = returnedData[1];
          handleNewText(newPath, newText);
        });
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_PATH')
        .listen((um) => absolutePathPrefix = um.body);
    
    newButton.onClick.listen((e) {
      var newPath = absolutePathPrefix + '/untitled.cc';
      var newText = ROS_TALKER;
      handleNewText(newPath, newText);

      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });

    saveButton.onClick.listen((e) => saveText());
    
    themeButton.onClick.listen((e) {
      String newTheme = (aceEditor.theme.name == 'solarized_dark') ? Theme.SOLARIZED_LIGHT : Theme.SOLARIZED_DARK;
      aceEditor.theme = new Theme.named(newTheme);
      
      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });
  }
  
  /// Handles changes to the Editor model, new files and opening files.
  handleNewText(String newPath, String newText) {
    if (noUnsavedChanges()) {
      setEditorText(newPath, newText);
    } else {
      presentModal();
      modalSaveButton.onClick.listen((e) {
        saveText();
        setEditorText(newPath, newText);
      });
      modalDiscardButton.onClick.listen((e) {
        setEditorText(newPath, newText);
      });
    }
  }
  
  /// Sets the Editor's text with [newText], updates [openFilePath], and resets the save point.
  setEditorText(String newPath, String newText) {
    openFilePath = newPath;
    aceEditor.setValue(newText, 1);
    resetSavePoint();
    
    // Set focus to the interactive area so the user can typing immediately.
    aceEditor.focus();
  }
  
  /// Shows the modal for unsaved changes.
  void presentModal() {
    DivElement modal = querySelector('#myModal');
    Modal m = new Modal(modal);
    m.show();
  }
  
  /// Sends the file path and contents to the server to be saved to disk.
  void saveText() {
    ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFilePath);
    resetSavePoint();
  }
  
  /// Compares the Editor's current text with text at the last save point.
  bool noUnsavedChanges() => aceEditor.value == originalContents;
  
  /// Resets the save point based on the Editor's current text.
  String resetSavePoint() => originalContents = aceEditor.value;
}