library updroid_editor;

import 'dart:html';
import 'dart:async';
import 'package:dnd/dnd.dart';

import 'package:ace/ace.dart' as ace;
import 'package:bootjack/bootjack.dart';
import 'package:ace/proxy.dart';
import "package:path/path.dart" as pathLib;

import 'lib/updroid_message.dart';
import 'tab.dart';
import  'modal.dart';

part 'lib/editor/templates.dart';

/// [UpDroidEditor] is a wrapper for an embedded Ace Editor. Sets styles
/// for the editor and an additional menu bar with some filesystem operations.
class UpDroidEditor extends UpDroidTab {
  static const String className = 'UpDroidEditor';
  String type = 'UpDroidEditor';

  WebSocket _ws;
  StreamController<CommanderMessage> _cs;
  int num;
  int _col;

  Map _pathMap;
  String _absolutePathPrefix;

  DivElement _content;
  LIElement _fileName;
  AnchorElement _newButton;
  AnchorElement _saveButton;
  AnchorElement _saveAsButton;
  AnchorElement _closeTabButton;
  AnchorElement _themeButton;
  InputElement _fontSizeInput;

  Element _saveCommit;
  ButtonElement _modalSaveButton;
  ButtonElement _modalDiscardButton;
  Element _warning;
  Element _overwriteCommit;
  var _curModal;
  Dropzone linkedDropzone;

  int _fontSize = 12;

  // Stream Subscriptions.
  StreamSubscription _saveAsClickEnd;
  StreamSubscription _saveAsEnterEnd;
  StreamSubscription _unsavedSave;
  StreamSubscription _unsavedDiscard;
  StreamSubscription _overwrite;
  StreamSubscription _fontInputListener;

  ace.Editor _aceEditor;
  String _openFilePath;
  String _originalContents;
  String _currentParPath;

  UpDroidEditor(this.num, int col, StreamController<CommanderMessage> cs, {bool active: false}) {
    _col = col;
    _cs = cs;

    setUpTabHandle(num, _col, 'Editor', active);
    setUpTabContainer(num, _col, 'Editor', _getMenuConfig(), active).then((Map configRefs) {
      setUpUI(configRefs);

      _fontSizeInput.placeholder = _fontSize.toString();

      // Create the server <-> client [WebSocket].
      // Port 12060 is the default port that UpDroid uses.
      String url = window.location.host;
      url = url.split(':')[0];
      _ws = new WebSocket('ws://' + url + ':12060/editor/$num');

      _setUpEditor();
      _registerEditorEventHandlers();

      _cs.add(new CommanderMessage('EXPLORER', 'EDITOR_READY', body: [num, _content]));
    });
  }

  void setUpUI(Map configRefs) {
    _content = configRefs['content'];
    _fileName = configRefs['extra'];
    _newButton = configRefs['new'];
    _saveButton = configRefs['save'];
    _saveAsButton = configRefs['save-as'];
    _closeTabButton = configRefs['close-tab'];
    _themeButton = configRefs['theme'];
    _fontSizeInput = configRefs['font-size'];

  }

  /// Sets up the editor and styles.
  void _setUpEditor() {
    ace.implementation = ACE_PROXY_IMPLEMENTATION;

    _aceEditor = ace.edit(_content);
    _aceEditor
      ..session.mode = new ace.Mode.named(ace.Mode.PYTHON)
      ..fontSize = _fontSize
      ..theme = new ace.Theme.named(ace.Theme.SOLARIZED_DARK);

    // Necessary to allow our styling (in main.css) to override Ace's.
    _content.classes.add('updroid_editor');

    _resetSavePoint();
  }

  /// Process messages according to the type.
  void _processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CLASS_ADD':
        _content.classes.add(m.body);
        break;

      case 'CLASS_REMOVE':
        _content.classes.remove(m.body);
        break;

      case 'OPEN_FILE':
        if(num == m.body[0]) {
          _ws.send('[[EDITOR_OPEN]]' + m.body[1]);
          _fileName.text = pathLib.basename(m.body[1]);
        }
        break;

      case 'PARENT_PATH':
        _currentParPath = m.body;
        break;

      case 'PASS_EDITOR_INFO':
        if(num == m.body[0]){
          linkedDropzone = m.body[1];
        }
        break;

      default:
        print('Client error: unrecognized message type: ' + m.type);
    }
  }

  /// Sets up event handlers for the editor's menu buttons.
  void _registerEditorEventHandlers() {
    _cs.stream
        .where((m) => m.dest == 'EDITOR')
        .listen((m) => _processMessage(m));

    _ws.onOpen.listen((e) => _ws.send('[[EDITOR_DIRECTORY_PATH]]'));

    // Editor receives the open file contents from the server.
    _ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_FILE_TEXT')
        .listen((um) {
          var returnedData = um.body.split('[[CONTENTS]]');
          var newPath = returnedData[0];
          var newText = returnedData[1];
          _handleNewText(newPath, newText);
        });

    _ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_DIRECTORY_PATH')
        .listen((um) => _absolutePathPrefix = um.body);

    _ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EDITOR_NEW_FILENAME')
        .listen((um) {
          var newText = RosTemplates.templateCode;
          var newPath = _absolutePathPrefix + '/' + um.body;
          _handleNewText(newPath, newText);
        });

    _ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'PATH_LIST')
        .listen((um) => _pullPaths(um.body));

    _fontSizeInput.onClick.listen((e) {
      // Keeps bootjack dropdown from closing
      e.stopPropagation();

      _fontInputListener = _fontSizeInput.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          var fontVal;
          try {
            fontVal = int.parse(_fontSizeInput.value);
            assert(fontVal is int);
            if(fontVal >= 1 && fontVal <= 60){
              _aceEditor.fontSize = fontVal;
              _fontSizeInput.placeholder = fontVal.toString();
            }
          }
          finally {
            _fontSizeInput.value = "";
            querySelector('#editor').click();
            _aceEditor.focus();
            _fontInputListener.cancel();
          }
        }
      });
    });

    tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_col}_UpDroidEditor'));
    });

    _closeTabButton.onClick.listen((e) {
      destroyTab();
      _cs.add(new CommanderMessage('CLIENT', 'CLOSE_TAB', body: '${type}_$num'));
      _cs.add(new CommanderMessage('EXPLORER', 'REMOVE_EDITOR', body: linkedDropzone));
    });

    _newButton.onClick.listen((e) {
      _openFilePath = null;
      if (_noUnsavedChanges()) {
        _aceEditor.setValue(RosTemplates.templateCode, 1);
        _fileName.text = "untitled";
      }
      else{
        e.preventDefault();
        new UpDroidUnsavedModal();

        // TODO: refine this case
        _modalSaveButton = querySelector('.modal-save');
        _modalDiscardButton = querySelector('.modal-discard');

        _unsavedSave = _modalSaveButton.onClick.listen((e) {
          _saveText();
          _aceEditor.setValue(RosTemplates.templateCode, 1);
          _fileName.text = "untitled";
          _unsavedSave.cancel();
        });
        _unsavedDiscard = _modalDiscardButton.onClick.listen((e) {
          _aceEditor.setValue(RosTemplates.templateCode, 1);
          _fileName.text = "untitled";
          _unsavedDiscard.cancel();
        });
      }
      _aceEditor.focus();
      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });

    _saveButton.onClick.listen((e) => _saveText());


    /// Save as click handler

    _saveAsButton.onClick.listen((e) {
      _cs.add(new CommanderMessage('EXPLORER', 'REQUEST_PARENT_PATH'));
      _ws.send("[[EDITOR_REQUEST_LIST]]");
      if(_curModal != null) _curModal.hide();

      String saveAsPath = '';
      _curModal = new UpDroidSavedModal();
      var input = querySelector('#save-as-input');
      _saveCommit = querySelector('#save-as-commit');
      _overwriteCommit = querySelector('#warning button');
      _warning = querySelector('#warning');

      void completeSave() {
          _ws.send('[[EDITOR_SAVE]]' + _aceEditor.value + '[[PATH]]' + saveAsPath);
          _fileName.text = input.value;
          _curModal.hide();
          input.value = '';
          _resetSavePoint();
          _saveAsClickEnd.cancel();
          _saveAsEnterEnd.cancel();
          _openFilePath = saveAsPath;
      }

      // Check to make sure that the supplied input doesn't conflict with existing files
      // on system.  Also determines what action to take depending on whether the file exists or not.

      void _checkSave() {

        // User enters no input
        if (input.value == '') {
          window.alert("Please enter a valid filename");
        }

        // Determining the save path
        if (_openFilePath == null) {
          if(_currentParPath == null) {
            saveAsPath = pathLib.normalize(pathLib.normalize(_absolutePathPrefix+ '/src') + "/${input.value}");
          }
          else{
            saveAsPath = pathLib.normalize(_currentParPath + "/${input.value}");
          }
        }
        else {
          saveAsPath = pathLib.dirname(_openFilePath)+  "/${input.value}";
        }

        // Filename already exists on system
        if (_pathMap.containsKey(saveAsPath)) {
          if (_pathMap[saveAsPath] == 'directory') {
            window.alert("That filename already exists as a directory");
            input.value = "";
          }

          else if (_pathMap[saveAsPath] == 'file') {
            _warning.classes.remove('hidden');
            _overwrite = _overwriteCommit.onClick.listen((e){
              completeSave();
              _warning.classes.add('hidden');
              _overwrite.cancel();
            });
          }
        }

        // Filename clear, continue with save
        else {
          completeSave();
        }
      }

      _saveAsClickEnd = _saveCommit.onClick.listen((e) {
        _checkSave();
      });

      _saveAsEnterEnd = input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          _checkSave();
        }
      });
    });

    _themeButton.onClick.listen((e) {
      String newTheme = (_aceEditor.theme.name == 'solarized_dark') ? ace.Theme.SOLARIZED_LIGHT : ace.Theme.SOLARIZED_DARK;
      _aceEditor.theme = new ace.Theme.named(newTheme);

      // Stops the button from sending the page to the top (href=#).
      e.preventDefault();
    });
  }

  /// Handles changes to the Editor model, new files and opening files.
  _handleNewText(String newPath, String newText) {
    if (_noUnsavedChanges()) {
      _setEditorText(newPath, newText);
    } else {
      new UpDroidUnsavedModal();
      _modalSaveButton = querySelector('.modal-save');
      _modalDiscardButton = querySelector('.modal-discard');

      _unsavedSave = _modalSaveButton.onClick.listen((e) {
        _saveText();
        _setEditorText(newPath, newText);
        _unsavedSave.cancel();
      });
      _unsavedDiscard = _modalDiscardButton.onClick.listen((e) {
        _setEditorText(newPath, newText);
        _unsavedDiscard.cancel();
      });
    }
  }

  /// Sets the Editor's text with [newText], updates [_openFilePath], and resets the save point.
  _setEditorText(String newPath, String newText) {
    _openFilePath = newPath;
    _aceEditor.setValue(newText, 1);
    _resetSavePoint();

    // Set focus to the interactive area so the user can typing immediately.
    _aceEditor.focus();
  }

  /// Sends the file path and contents to the server to be saved to disk.
  void _saveText() {
    if (_openFilePath == null) {
      _saveAsButton.click();
    }
    else {
      _ws.send('[[EDITOR_SAVE]]' + _aceEditor.value + '[[PATH]]' + _openFilePath);
      _resetSavePoint();

    }
  }

  void _pullPaths(String raw) {
    var pathList;
    raw = raw.replaceAll(new RegExp(r"(\[|\]|')"), '');
    pathList = raw.split(',');
    _pathMap = new Map.fromIterable(pathList,
        key: (item) => item.replaceAll(new RegExp(r"(Directory: | File: |Directory: |File:)"), '').trim(),
        value: (item) => item.contains("Directory:") ? "directory" : "file"
        );
  }

  /// Compares the Editor's current text with text at the last save point.
  bool _noUnsavedChanges() => _aceEditor.value == _originalContents;

  /// Resets the save point based on the Editor's current text.
  String _resetSavePoint() => _originalContents = _aceEditor.value;

  List _getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'New'},
        {'type': 'toggle', 'title': 'Save'},
        {'type': 'toggle', 'title': 'Save As'},
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Theme'},
        {'type': 'input', 'title': 'Font Size'}]}
    ];
    return menu;
  }
}