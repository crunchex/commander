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
  AnchorElement saveAsButton;
  AnchorElement newButton;
  AnchorElement themeButton;
  ButtonElement modalSaveButton;
  ButtonElement modalDiscardButton;
  InputElement fontSizeInput;
  int fontSize = 16;
  StreamSubscription fontInputListener;
  Modal curModal;

  ace.Editor aceEditor;
  String openFilePath;
  String originalContents;

  UpDroidEditor(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;

    editorDiv = querySelector('#editor');

    saveButton = querySelector('#button-save');
    newButton = querySelector('#button-new');
    saveAsButton = querySelector('#button-save-as');
    themeButton = querySelector('#button-editor-theme');
    modalSaveButton = querySelector('.modal-save');
    modalDiscardButton = querySelector('.modal-discard');

    fontSizeInput = querySelector("#font-size-input");
    fontSizeInput.placeholder = fontSize.toString();

    setUpEditor();
    registerEditorEventHandlers();

    cs.add(new CommanderMessage('CLIENT', 'EDITOR_READY'));
  }

  /// Sets up the editor and styles.
  void setUpEditor() {
    ace.implementation = ACE_PROXY_IMPLEMENTATION;

    aceEditor = ace.edit(editorDiv);
    aceEditor
      ..session.mode = new ace.Mode.named(ace.Mode.PYTHON)
      ..fontSize = fontSize
      ..theme = new ace.Theme.named(ace.Theme.SOLARIZED_DARK);

    resetSavePoint();
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CLASS_ADD':
        editorDiv.classes.add(m.body);
        break;

      case 'CLASS_REMOVE':
        editorDiv.classes.remove(m.body);
        break;

      case 'OPEN_FILE':
        ws.send('[[EDITOR_OPEN]]' + m.body);
        break;

      default:
        print('Client error: unrecognized message type: ' + m.type);
    }
  }

  /// Sets up event handlers for the editor's menu buttons.
  void registerEditorEventHandlers() {
    cs.stream
        .where((m) => m.dest == 'EDITOR')
        .listen((m) => processMessage(m));

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

    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_NEW_FILENAME')
        .listen((um) {
          var newText = ROS_TALKER;
          var newPath = absolutePathPrefix + '/' + um.body;
          handleNewText(newPath, newText);
        });

    fontSizeInput.onClick.listen((e){

      // Keeps bootjack dropdown from closing
      e.stopPropagation();

     fontInputListener = fontSizeInput.onKeyUp.listen((e) {
      var keyEvent = new KeyEvent.wrap(e);
              if (keyEvent.keyCode == KeyCode.ENTER) {
                var fontVal;
                try{
                  fontVal = int.parse(fontSizeInput.value);
                  assert(fontVal is int);
                  if(fontVal >= 1 && fontVal <= 60){
                    aceEditor.fontSize = fontVal;
                    fontSizeInput.placeholder = fontVal.toString();
                  }
                }
                finally{
                  fontSizeInput.value = "";
                  querySelector('#editor').click();
                  aceEditor.focus();
                  fontInputListener.cancel();
                }
              }
      });
    });

    newButton.onClick.listen((e) {
      // Editor needs to request an available filename (e.g. untitled.py, untitled1.py, etc.)

      ws.send('[[EDITOR_REQUEST_FILENAME]]' + absolutePathPrefix);

      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });

    saveButton.onClick.listen((e) => saveText());

    saveAsButton.onClick.listen((e){
      var input = querySelector('#save-as-input');
      var saveCommit = querySelector('#save-as-commit');
      var close = querySelector('.close');
      presentModal("#save-as");
      input.onKeyUp.listen((e){
        bool completed = false;
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER){
          input = querySelector('#save-as-input');
          if(input.value == '' ) {
            saveText();
          }
          else if(openFilePath == null){
            ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + absolutePathPrefix + "/" + input.value);
          }
          input.value = "";
          curModal.hide();
        }
      });
    });

    themeButton.onClick.listen((e) {
      String newTheme = (aceEditor.theme.name == 'solarized_dark') ? ace.Theme.SOLARIZED_LIGHT : ace.Theme.SOLARIZED_DARK;
      aceEditor.theme = new ace.Theme.named(newTheme);

      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });
  }

  /// Handles changes to the Editor model, new files and opening files.
  handleNewText(String newPath, String newText) {
    if (noUnsavedChanges()) {
      setEditorText(newPath, newText);
    } else {
      presentModal("#unsaved");
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
  void presentModal(String selector) {
    DivElement modal = querySelector(selector);
    curModal = new Modal(modal);
    curModal.show();
  }

  /// Sends the file path and contents to the server to be saved to disk.
  void saveText() {
    if (openFilePath == null) {
      saveAsButton.click();
    }
    else{
      ws.send('[[EDITOR_SAVE]]' + aceEditor.value + '[[PATH]]' + openFilePath);
      resetSavePoint();

    }
  }

  /// Compares the Editor's current text with text at the last save point.
  bool noUnsavedChanges() => aceEditor.value == originalContents;

  /// Resets the save point based on the Editor's current text.
  String resetSavePoint() => originalContents = aceEditor.value;
}