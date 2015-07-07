library updroid_editor;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

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

  // Stream Subscriptions.
  StreamSubscription _fontInputListener;
  StreamSubscription _fileChangesListener;

  ace.Editor _aceEditor;
  String _openFilePath;
  bool _exec;
  var _curModal;
  int _fontSize = 12;
  String _originalContents;

  UpDroidEditor(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Editor', getMenuConfig(), cs, true) {

  }

  void setUpController() {
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

    _openFilePath = null;
    _exec = false;
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

    _fileChangesListener = _aceEditor.onChange.listen((e) => _updateUnsavedChangesIndicator());
  }

  // Mailbox Handlers

  /// Editor receives the open file contents from the server.
  void _openFileHandler(UpDroidMessage um) {
    List<String> returnedData = um.body.split('[[CONTENTS]]');
    String newPath = returnedData[0];
    String newText = returnedData[1];

    _handleAnyChanges().then((_) {
      _updateOpenFilePath(newPath);
      _setEditorText(newText);
    });
  }

  // Event Handlers

  void _newFileHandler(Event e, String newText) {
    e.preventDefault();

    _handleAnyChanges().then((_) {
      _updateOpenFilePath(null);
      _setEditorText(newText);
    });
  }

  void _saveHandler() {
    if (_noUnsavedChanges()) return;

    if (_openFilePath != null) {
      _saveFile();
      return;
    }

    // _openFilePath is null, so we need to run the Save-As routine.
    _updatePathAndExec().then((bool completeSave) {
      if (completeSave) _saveFile();
    });
  }

  void _saveAsHandler() {
    _updatePathAndExec().then((bool completeSave) {
      if (completeSave) _saveFile();
    });
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

  /// Adds an asterisk to the displayed filename if there are any unsaved changes.
  void _updateUnsavedChangesIndicator() {
    if (_noUnsavedChanges()) return;

    if (!view.extra.text.contains('*')) view.extra.text = view.extra.text + '*';
  }

  // Misc Private Methods

  Future _handleAnyChanges() {
    Completer c = new Completer();

    if (_noUnsavedChanges()) {
      c.complete();
    } else {
      _presentUnsavedChangesModal().then((bool continueSave) {
        if (!continueSave) {
          c.complete();
        } else {
          _updatePathAndExec().then((bool completeSave) {
            if (completeSave) _saveFile();
            c.complete();
          });
        }
      });
    }

    return c.future;
  }

  Future<bool> _updatePathAndExec() {
    Completer c = new Completer();

    _getSelectedPath().then((String path) {
      if (path == null) {
        window.alert('Please choose one directory from Explorer and retry.');
        c.complete(false);
      } else {
        _presentSaveAsModal(path).then((bool completeSave) {
          c.complete(completeSave);
        });
      }
    });

    return c.future;
  }

  Future<String> _getSelectedPath() {
    Completer c = new Completer();

    mailbox.waitFor(new UpDroidMessage('REQUEST_SELECTED', '')).then((UpDroidMessage um) {
      print(um.body);
      List<String> selectedPaths = JSON.decode(um.body);
      if (selectedPaths.length != 1) {
        c.complete(null);
      } else {
        c.complete(pathLib.normalize(selectedPaths[0]));
      }
    });

    return c.future;
  }

  Future<bool> _presentUnsavedChangesModal() {
    Completer c = new Completer();

    if (_curModal != null) {
      _curModal.hide();
      _curModal = null;
    }

    _curModal = new UpDroidUnsavedModal();
    List<StreamSubscription> subs = [];

    subs.add(_curModal.saveButton.onClick.listen((e) {
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      c.complete(true);
    }));

    subs.add(_curModal.discardButton.onClick.listen((e) {
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      c.complete(false);
    }));

    return c.future;
  }

  Future<bool> _presentSaveAsModal(path) {
    Completer c = new Completer();

    if (_curModal != null) {
      _curModal.hide();
      _curModal = null;
    }

    _curModal = new UpDroidSavedModal();
    List<StreamSubscription> subs = [];

    subs.add(_curModal.saveButton.onClick.listen((e) {
      if (_curModal.input.value == '') return;

      _updateOpenFilePath(pathLib.normalize(path + '/' + _curModal.input.value));
      _exec = _curModal.makeExec.checked;
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      c.complete(true);
    }));

    subs.add(_curModal.discardButton.onClick.listen((e) {
      subs.forEach((StreamSubscription sub) => sub.cancel());
      _curModal.hide();
      c.complete(false);
    }));

    return c.future;
  }

  void _saveFile() {
    if (_openFilePath == null || _openFilePath == '') return;

    mailbox.ws.send('[[SAVE_FILE]]' + JSON.encode([_aceEditor.value, _openFilePath, _exec]));

    view.extra.text = pathLib.basename(_openFilePath);
    _resetSavePoint();
  }

  /// Sets the Editor's text with [newText], updates [_openFilePath], and resets the save point.
  void _setEditorText(String newText) {
    _aceEditor.setValue(newText, 1);
    _exec = false;
    _resetSavePoint();

    // Set focus to the interactive area so the user can typing immediately.
    _aceEditor.focus();
    _aceEditor.scrollToLine(0);
  }

  /// Compares the Editor's current text with text at the last save point.
  bool _noUnsavedChanges() => _aceEditor.value == _originalContents;

  /// Resets the save point based on the Editor's current text.
  String _resetSavePoint() => _originalContents = _aceEditor.value;

  void _updateOpenFilePath(String newPath) {
    _openFilePath = newPath;

    if (_openFilePath == null || _openFilePath == '') {
      view.extra.text = 'untitled';
    } else {
      view.extra.text = pathLib.basename(_openFilePath);
    }

    view.tabHandleButton.title = view.extra.text;
  }

  Future<bool> preClose() {
    return _handleAnyChanges().then((_) => true);
  }

  void cleanUp() {
    _fileChangesListener.cancel();
  }
}