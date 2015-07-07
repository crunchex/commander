library updroid_editor;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:dnd/dnd.dart';

import 'package:ace/ace.dart' as ace;
import 'package:ace/proxy.dart';
import "package:path/path.dart" as pathLib;

import '../../mailbox.dart';
import '../tab_controller.dart';
import '../../modal/modal.dart';

part 'templates.dart';

/// [UpDroidEditor] is a wrapper for an embedded Ace Editor. Sets styles
/// for the editor and an additional menu bar with some filesystem operations.
class UpDroidEditor extends TabController {
  static String className = 'UpDroidEditor';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'submenu', 'title': 'New...', 'items':
          ['Blank', 'Publisher', 'Subscriber', 'Basic Launch File']},
        {'type': 'submenu', 'title': 'Examples', 'items':
          ['Hello World Talker', 'Hello World Listener']},
        {'type': 'toggle', 'title': 'Save'},
        {'type': 'toggle', 'title': 'Save As'},
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Invert'},
        {'type': 'input', 'title': 'Font Size'}]}
    ];
    return menu;
  }

  Map _pathMap;
  String _absolutePathPrefix;

  AnchorElement _blankButton;
  AnchorElement _launchButton;
  AnchorElement _talkerButton;
  AnchorElement _listenerButton;
  AnchorElement _pubButton;
  AnchorElement _subButton;
  AnchorElement _saveButton;
  AnchorElement _saveAsButton;
  AnchorElement _themeButton;
  InputElement _fontSizeInput;

  ButtonElement _modalSaveButton;
  ButtonElement _modalDiscardButton;
  Element _warning;
  Element _overwriteCommit;
  DivElement _explorersDiv;
  var _curModal;
  int _fontSize = 12;

  // Stream Subscriptions.
  StreamSubscription _saveAsClickEnd;
  StreamSubscription _saveAsEnterEnd;
  StreamSubscription _unsavedSave;
  StreamSubscription _unsavedDiscard;
  StreamSubscription _overwrite;
  StreamSubscription _fontInputListener;
  StreamSubscription _fileChangesListener;

  ace.Editor _aceEditor;
  String _openFilePath;
  String _originalContents;

  UpDroidEditor(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Editor', getMenuConfig(), cs, true) {

  }

  void setUpController() {
    _explorersDiv = querySelector('#exp-container');
    _blankButton = view.refMap['blank-button'];
    _pubButton = view.refMap['publisher-button'];
    _subButton = view.refMap['subscriber-button'];
    _launchButton = view.refMap['basic-launch-file-button'];
    _talkerButton = view.refMap['hello-world-talker-button'];
    _listenerButton = view.refMap['hello-world-listener-button'];
    _saveButton = view.refMap['save'];
    _saveAsButton = view.refMap['save-as'];
    _themeButton = view.refMap['invert'];
    _fontSizeInput = view.refMap['font-size'];
    _fontSizeInput.placeholder = _fontSize.toString();

    ace.implementation = ACE_PROXY_IMPLEMENTATION;
    ace.BindKey ctrlS = new ace.BindKey(win: "Ctrl-S", mac: "Command-S");
    ace.Command save = new ace.Command('save', ctrlS, (d) => _saveHandler());

    DivElement aceDiv = new DivElement()
    // Necessary to allow our styling (in main.css) to override Ace's.
      ..classes.add('updroid_editor');
    view.content.children.add(aceDiv);

    _aceEditor = ace.edit(aceDiv)
      ..session.mode = new ace.Mode.named(ace.Mode.PYTHON)
      ..fontSize = _fontSize
      ..theme = new ace.Theme.named(ace.Theme.SOLARIZED_DARK)
      ..commands.addCommand(save);

    _resetSavePoint();
  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'OPEN_FILE', _openFileHandler);
  }

  void registerEventHandlers() {
    _blankButton.onClick.listen((e) => _newFileHandler(e, ''));
    _talkerButton.onClick.listen((e) => _newFileHandler(e, RosTemplates.talkerTemplate));
    _listenerButton.onClick.listen((e) => _newFileHandler(e, RosTemplates.listenerTemplate));
    _launchButton.onClick.listen((e) => _newFileHandler(e, RosTemplates.launchTemplate));
    _pubButton.onClick.listen((e) => _newFileHandler(e, RosTemplates.pubTemplate));
    _subButton.onClick.listen((e) => _newFileHandler(e, RosTemplates.subTemplate));

    _saveButton.onClick.listen((e) => _saveHandler());
    _saveAsButton.onClick.listen((e) => _saveAsHandler());

    _themeButton.onClick.listen((e) => _invertTheme(e));
    _fontSizeInput.onClick.listen((e) => _updateFontSize(e));

    // Create listener to indicate that there are unsaved changes when file is altered.
    _fileChangesListener = _aceEditor.onChange.listen((e) {
      if (_noUnsavedChanges()) return;
      view.extra.text = _openFilePath == null ? 'untitled*' : pathLib.basename(_openFilePath) + '*';
    });
  }

  // Mailbox Handlers

  /// Editor receives the open file contents from the server.
  void _openFileHandler(UpDroidMessage um) {
    var returnedData = um.body.split('[[CONTENTS]]');
    var newPath = returnedData[0];
    var newText = returnedData[1];

    _dealWithUnsavedChanges().then((_) => _setEditorText(newPath, newText));
  }

  // Event Handlers

  void _newFileHandler(Event e, String text) {
    e.preventDefault();

    _aceEditor.setOptions({'readOnly' : false});
    _openFilePath = null;

    _dealWithUnsavedChanges().then((_) {
      _aceEditor.setValue(text, 1);
      view.extra.text = "untitled*";
      _aceEditor.focus();
    });
  }

  void _saveHandler() {
    if (_noUnsavedChanges()) return;

    if (_openFilePath == null) {
      _presentSaveAsModal();
    } else {
      _saveFile(_openFilePath, false);
    }
  }

  void _saveAsHandler() {
    if (_noUnsavedChanges()) return;
    _presentSaveAsModal();
  }

  void _invertTheme(Event e) {
    // Stops the button from sending the page to the top (href=#).
    e.preventDefault();

    bool dark = _aceEditor.theme.name == 'solarized_dark';
    String newTheme = dark ? ace.Theme.SOLARIZED_LIGHT : ace.Theme.SOLARIZED_DARK;
    _aceEditor.theme = new ace.Theme.named(newTheme);
  }

  void _updateFontSize(Event e) {
    // Keeps bootjack dropdown from closing
    e.stopPropagation();

    _fontInputListener = _fontSizeInput.onKeyUp.listen((e) {
      if (e.keyCode != KeyCode.ENTER) return;

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
        _aceEditor.focus();
        _fontInputListener.cancel();
      }
    });
  }

  // Misc Private Methods

  Future<String> _checkSave(String filename) {
    Completer c = new Completer();

    // Input field empty.
    if (filename == '') {
      window.alert("Please enter a valid filename");
      c.complete(null);
    } else {
      mailbox.waitFor(new UpDroidMessage('REQUEST_SELECTED', '')).then((UpDroidMessage um) {
        List<String> selectedPaths = JSON.decode(um.body);
        if (selectedPaths.length != 1 || selectedPaths[0] == '') {
          window.alert('Please select one directory from Explorer to save to and retry.');
          c.complete(null);
        }

        // TODO: add check for existing file for an "overwrite?" prompt.
        // else if () {}

        else {
          c.complete(pathLib.normalize('${selectedPaths[0]}/$filename'));
        }
      });
    }

    return c.future;
  }

  /// Handles changes to the Editor model, new files and opening files.
  Future _dealWithUnsavedChanges() {
    Completer c = new Completer();

    if (_noUnsavedChanges()) {
      c.complete();
    } else {
      if (_curModal != null) {
        _curModal.hide();
        _curModal = null;
      }

      _curModal = new UpDroidUnsavedModal();
      List<StreamSubscription> subs = [];

      subs.add(_curModal.saveButton.onClick.listen((e) {
        subs.forEach((StreamSubscription sub) => sub.cancel());
        _curModal.hide();
        _saveHandler();
        c.complete();
      }));

      subs.add(_curModal.discardButton.onClick.listen((e) {
        subs.forEach((StreamSubscription sub) => sub.cancel());
        _curModal.hide();
        c.complete();
      }));
    }

    return c.future;
  }

  Future _presentSaveAsModal() {
    Completer c = new Completer();

    if (_curModal != null) {
      _curModal.hide();
      _curModal = null;
    }

    _curModal = new UpDroidSavedModal();
    List<StreamSubscription> subs = [];

    subs.add(_curModal.saveButton.onClick.listen((e) {
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      _checkSave(_curModal.input.value).then((String path) {
        _saveFile(path, _curModal.makeExec.checked);
        c.complete();
      });
    }));

    subs.add(_curModal.discardButton.onClick.listen((e) {
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      c.complete();
    }));

    return c.future;
  }

  void _saveFile(String path, bool exec) {
    if (path == null || path == '') return;

    _openFilePath = path;
    _resetSavePoint();
    mailbox.ws.send('[[SAVE_FILE]]' + JSON.encode([_aceEditor.value, _openFilePath, exec]));
    view.extra.text = pathLib.basename(_openFilePath);
  }

  /// Sets the Editor's text with [newText], updates [_openFilePath], and resets the save point.
  void _setEditorText(String newPath, String newText) {
    view.extra.text = pathLib.basename(newPath);
    _openFilePath = newPath;
    _aceEditor.setValue(newText, 1);
    _resetSavePoint();

    // Set focus to the interactive area so the user can typing immediately.
    _aceEditor.focus();
    _aceEditor.scrollToLine(0);
  }

  /// Compares the Editor's current text with text at the last save point.
  bool _noUnsavedChanges() => _aceEditor.value == _originalContents;

  /// Resets the save point based on the Editor's current text.
  String _resetSavePoint() => _originalContents = _aceEditor.value;

  Future<bool> preClose() {
    return _dealWithUnsavedChanges().then((_) => true);
  }

  void cleanUp() {
    _fileChangesListener.cancel();
  }
}