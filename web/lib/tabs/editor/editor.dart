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

  Element _saveCommit;
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
  String _currentParPath;
  bool _exec;

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
    _blankButton.onClick.listen((e) => _handleNewFileButton(e, ''));
    _talkerButton.onClick.listen((e) => _handleNewFileButton(e, RosTemplates.talkerTemplate));
    _listenerButton.onClick.listen((e) => _handleNewFileButton(e, RosTemplates.listenerTemplate));
    _launchButton.onClick.listen((e) => _handleNewFileButton(e, RosTemplates.launchTemplate));
    _pubButton.onClick.listen((e) => _handleNewFileButton(e, RosTemplates.pubTemplate));
    _subButton.onClick.listen((e) => _handleNewFileButton(e, RosTemplates.subTemplate));

    _saveButton.onClick.listen((e) => _saveHandler());
    _saveAsButton.onClick.listen((e) => _saveAsHandler());

    _themeButton.onClick.listen((e) => _invertTheme(e));
    _fontSizeInput.onClick.listen((e) => _updateFontSize(e));

    // Create listener to indicate that there are unsaved changes when file is altered
    _fileChangesListener = _aceEditor.onChange.listen((e) {
      if (_openFilePath != null && _noUnsavedChanges() == false) {
        view.extra.text = pathLib.basename(_openFilePath) + '*';
      }
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

  void _saveHandler() {
    if (_openFilePath == null) {
      _saveAsHandler();
      return;
    }

    if (pathLib.basename(_openFilePath) == 'CMakeLists.txt') return;

    mailbox.ws.send('[[SAVE_FILE]]' + JSON.encode([_aceEditor.value, _openFilePath, false]));
    _resetSavePoint();
    view.extra.text = pathLib.basename(_openFilePath);
  }

  void _saveAsHandler() {
    _exec = false;
    mailbox.ws.send("[[EDITOR_REQUEST_LIST]]");
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

  void _saveFile() {
    if (_curModal != null) _curModal.hide();

    String saveAsPath = '';
    _curModal = new UpDroidSavedModal();

    //TODO: remove query selectors
    InputElement input = querySelector('#save-as-input');
    CheckboxInputElement makeExec = querySelector('#make-exec');
    _saveCommit = querySelector('#save-as-commit');
    _overwriteCommit = querySelector('#warning button');
    _warning = querySelector('#warning');

    // Check to make sure that the supplied input doesn't conflict with existing files
    // on system.  Also determines what action to take depending on whether the file exists or not.

    _saveAsClickEnd = _saveCommit.onClick.listen((e) {
      if (makeExec.checked == true) _exec = true;
      _checkSave(input, saveAsPath);
    });

    _saveAsEnterEnd = input.onKeyUp.listen((e) {
      if (e.keyCode != KeyCode.ENTER) return;

      if (makeExec.checked == true) _exec = true;
      _checkSave(input, saveAsPath);
    });
  }

  void _completeSave(InputElement input, String saveAsPath) {
    mailbox.ws.send('[[SAVE_FILE]]' + JSON.encode([_aceEditor.value, saveAsPath, _exec]));

    view.extra.text = input.value;
    _curModal.hide();
    input.value = '';
    _resetSavePoint();
    _saveAsClickEnd.cancel();
    _saveAsEnterEnd.cancel();
    _openFilePath = saveAsPath;
  }

  void _checkSave(InputElement input, String saveAsPath) {
    // User enters no input
    if (input.value == '') window.alert("Please enter a valid filename");

    // Determining the save path
    if (_openFilePath == null) {
      if(_currentParPath == null) {
        var activeFolderName = _checkActiveExplorer();
        saveAsPath = pathLib.normalize(pathLib.normalize(_absolutePathPrefix + '/' + activeFolderName +'/src') + "/${input.value}");
      } else{
        saveAsPath = pathLib.normalize(_currentParPath + "/${input.value}");
      }
    }
    else {
      if(_currentParPath == null) saveAsPath = pathLib.dirname(_openFilePath)+  "/${input.value}";
      else {
        saveAsPath = pathLib.normalize(_currentParPath + "/${input.value}");
      }
    }

    // Filename already exists on system
    if (_pathMap.containsKey(saveAsPath)) {
      if (_pathMap[saveAsPath] == 'directory') {
        window.alert("That filename already exists as a directory");
        input.value = "";
      } else if (_pathMap[saveAsPath] == 'file') {
        _warning.classes.remove('hidden');
        _overwrite = _overwriteCommit.onClick.listen((e){
          _completeSave(input, saveAsPath);
          _warning.classes.add('hidden');
          _overwrite.cancel();
        });
      }
    }

    // Filename clear, continue with save
    else {
      _completeSave(input, saveAsPath);
    }
  }

  void _handleNewFileButton(Event e, String text) {
    e.preventDefault();

    _aceEditor.setOptions({'readOnly' : false});
    _openFilePath = null;

    if (_noUnsavedChanges()) {
      _aceEditor.setValue(text, 1);
      view.extra.text = "untitled*";
      _aceEditor.focus();
      return;
    }

    new UpDroidUnsavedModal();

    // TODO: refine this case
    _modalSaveButton = querySelector('.modal-save');
    _modalDiscardButton = querySelector('.modal-discard');

    _unsavedSave = _modalSaveButton.onClick.listen((e) {
      _saveHandler();
      _aceEditor.setValue(text, 1);
      view.extra.text = "untitled*";
      _unsavedSave.cancel();
    });

    _unsavedDiscard = _modalDiscardButton.onClick.listen((e) {
      _aceEditor.setValue(text, 1);
      view.extra.text = "untitled*";
      _unsavedDiscard.cancel();
    });
  }

  /// Handles changes to the Editor model, new files and opening files.
  Future _dealWithUnsavedChanges() {
    Completer c = new Completer();

    if (_noUnsavedChanges()) {
      c.complete();
    } else {
      List<StreamSubscription> subs = [];
      UpDroidUnsavedModal modal = new UpDroidUnsavedModal();

      subs.add(modal.saveButton.onClick.listen((e) {
        subs.forEach((StreamSubscription sub) => sub.cancel());
        _saveHandler();
        c.complete();
      }));

      subs.add(modal.discardButton.onClick.listen((e) {
        subs.forEach((StreamSubscription sub) => sub.cancel());
        c.complete();
      }));
    }

    return c.future;
  }

  /// Sets the Editor's text with [newText], updates [_openFilePath], and resets the save point.
  void _setEditorText(String newPath, String newText) {
    view.extra.text = newPath.split('/').last;
    _openFilePath = newPath;
    _aceEditor.setValue(newText, 1);
    _resetSavePoint();

    // Set focus to the interactive area so the user can typing immediately.
    _aceEditor.focus();
    _aceEditor.scrollToLine(0);
  }

  /// parses through DOM to get the current active explorer
  String _checkActiveExplorer() {
    String activeName;
    for (var explorer in _explorersDiv.children) {
      if (explorer.id != 'recycle' && !explorer.classes.contains('control-buttons')) {
        if (!explorer.classes.contains('hidden')) {
          activeName = explorer.dataset['name'];
        }
      }
    }
    return activeName;
  }

  /// Compares the Editor's current text with text at the last save point.
  bool _noUnsavedChanges() => _aceEditor.value == _originalContents;

  /// Resets the save point based on the Editor's current text.
  String _resetSavePoint() => _originalContents = _aceEditor.value;

  Future<bool> preClose() async {
    Completer c = new Completer();

    if (_noUnsavedChanges()) {
      c.complete(true);
    } else {
      new UpDroidUnsavedModal();

      _modalSaveButton = querySelector('.modal-save');
      _modalDiscardButton = querySelector('.modal-discard');

      _unsavedSave = _modalSaveButton.onClick.listen((e) {
        _saveHandler();
        _unsavedSave.cancel();
        c.complete(true);
      });

      _unsavedDiscard = _modalDiscardButton.onClick.listen((e) {
        _unsavedDiscard.cancel();
        c.complete(true);
      });
    }

    return c.future;
  }

  void cleanUp() {
    _fileChangesListener.cancel();
  }
}