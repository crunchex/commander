library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'panels/explorer/explorer.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'mailbox.dart';

class UpDroidClient {
  StreamController<CommanderMessage> _cs;

  List<List<dynamic>> _columns;
  String _config;

  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;
  DivElement _explorersDiv;

  bool disconnectAlert = false;

  Mailbox _mailbox;
  bool _controlButtonEnabled;

  UpDroidClient() {
    _config = _getConfig();

    _columns = [[], [], []];

    _explorersDiv = querySelector('#exp-container');
    _newButtonLeft = querySelector('#column-1-new');
    _newButtonRight = querySelector('#column-2-new');

    _controlButtonEnabled = true;

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    _cs = new StreamController<CommanderMessage>.broadcast();
    _mailbox = new Mailbox('UpDroidClient', 1, _cs);

    _registerMailbox();
    _registerEventHandlers(_getConfig());

    _pulseFeedback(querySelector('#feedback-button'));
  }

  void _pulseFeedback(AnchorElement feedbackButton) {
    // Initial pulse - 30 seconds in.
    new Timer(new Duration(seconds: 30), () {
      feedbackButton.classes.add('feedback-bold');
      new Timer(new Duration(milliseconds: 500), () {
        feedbackButton.classes.remove('feedback-bold');
      });
    });

    // Every 5 minutes after.
    new Timer.periodic(new Duration(minutes: 5), (timer) {
      feedbackButton.classes.add('feedback-bold');
      new Timer(new Duration(milliseconds: 500), () {
        feedbackButton.classes.remove('feedback-bold');
      });
    });
  }

  /// Returns a [Map] of all the tabs that [UpDroidClient] needs to spawn,
  /// where key = side|left|right| and value = a list of IDs and UpDroidTab.className.
  /// TODO: this function should retrieve information from some defaults or
  /// a user account settings save.
  String _getConfig([String strConfig = '']) {
    // TODO: the default should eventually be JSON'd.
    //if (strConfig == null) strConfig = UpDroidClient.defaultConfig;
    if (strConfig != '') return strConfig;

    List listConfig = [
      [{'id': 1, 'class': 'UpDroidExplorer'}],
      [{'id': 1, 'class': 'UpDroidEditor'}],
      [{'id': 1, 'class': 'UpDroidConsole'}]
    ];

    return JSON.encode(listConfig);
  }

  int _getAvailableId(String className) {
    List ids = [];

    // Add all used ids for [className] to ids.
    for (int i = 1; i <= 2; i++) {
      _columns[i].forEach((tab) {

        if (tab.tabType == className) ids.add(tab.id);
      });
    }

    // Find the lowest unused ID possible.
    int id = 0;
    bool found = false;
    while (!found) {
      id++;
      if (!ids.contains(id)) break;
    }

    return id;
  }

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  void _initializeClient(UpDroidMessage um) {
    List config = JSON.decode(_config);

    for (int i = 0; i < 1; i++) {
      for (Map panel in config[i]) {
        _openPanel(i, panel['id'], panel['class']);
      }
    }

    for (int i = 1; i < config.length; i++) {
      for (Map tab in config[i]) {
        _openTab(i, tab['id'], tab['class']);
      }
    }
  }

  void _alertDisconnect(CommanderMessage m) {
    if (disconnectAlert == false) {
      window.alert("UpDroid Commander has lost connection to the server.");
      disconnectAlert = true;
    }
  }

  void _openPanel(int column, int id, String className) {
    if (_columns[column].length >= 1) return;

    if (_columns[column].isNotEmpty) {
      for (var panel in _columns[column]) {
        panel.makeInactive();
      }
    }

    if (className == 'UpDroidExplorer') {
      _mailbox.ws.send('[[OPEN_PANEL]]' + '$column-$id-$className');
       _columns[column].add(new UpDroidExplorer(id, column, _cs));
    }
  }

  void _openTab(int column, int id, String className) {
    if (_columns[column].length >= 4) return;

    if (_columns[column].isNotEmpty) {
      for (var tab in _columns[column]) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidEditor(id, column, _cs));
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidCamera(id, column));
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidTeleop(id, column));
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _columns[column].add(new UpDroidConsole(id, column, _cs));
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void _closeTab(CommanderMessage m) {
    String id = m.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Find the tab to remove and remove it.
    // Also take note of the column it was found in.
    int col;
    for (int i = 1; i <= 2; i++) {
      for (int j = 0; j < _columns[i].length; j++) {
        if (_columns[i][j].tabType == type && _columns[i][j].id == num) {
          _columns[i].removeAt(j);
          col = i;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    for (int j = 1; j < _columns[col].length; j++) {
      _columns[col][j].makeInactive();
    }

    if (_columns[col].length > 0) _columns[col].last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + id);
  }

  void _closeTabFromServer(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Find the tab to remove and remove it.
    // Also take note of the column it was found in.
    int col;
    for (int i = 0; i <= 1; i++) {
      for (int j = 0; j < _columns[i].length; j++) {
        if (_columns[i][j].tabType == type && _columns[i][j].id == num) {
          _columns[i].removeAt(j);
          col = i;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    for (int j = 0; j < _columns[col].length; j++) {
      _columns[col][j].makeInactive();
    }

    if (_columns[col].length > 0) _columns[col].last.makeActive();
  }

  void _openTabFromButton(CommanderMessage m) {
    List idList = m.body.split('_');
    int column = int.parse(idList[0]);
    String className = idList[1];

    int id = _getAvailableId(className);
    _openTab(column, id, className);
  }

  void _gitPassword(CommanderMessage m) {
    _mailbox.ws.send('[[GIT_PUSH]]' + '${_columns[0].first.currentSelectedPath}++${m.body}');
  }

  void _addWorkspace(CommanderMessage m) {
    String name;

    void complete() {
      var newNum = 1;
      var nums = [];
      var names = [];
      for (var explorer in _columns[0]) {
        nums.add(explorer.expNum);
        names.add(explorer.name);
      }
      while(nums.contains(newNum)){
        newNum ++;
      }

      if (names.contains(name)) {
        var prefix = name;
        num suffix = 1;
        while (names.contains(name)) {
          name = prefix + "_" + suffix.toString();
          suffix ++;
        }
      }
      _mailbox.ws.send('[[ADD_EXPLORER]]' + JSON.encode([newNum.toString(), name]));
//      _openExplorer(newNum, name);
      _cs.add(new CommanderMessage('UPDROIDEDITOR', 'RESEND_DROP'));
      if(_explorersDiv.classes.contains('hidden')) {
        DivElement control = querySelector('#control');
        _explorersDiv.classes.remove('hidden');
        control.classes.add('hidden');
      }
    }

    var workspaceModal = new UpDroidWorkspaceModal();
    List eles = workspaceModal.passRefs();
    var input = eles[0];
    var save = eles[1];
    save.onClick.listen((e){
      if(input.value == "") {
        name = "untitled";
      }
      else {
        name = input.value;
      }
      complete();
    });

    input.onKeyUp.listen((e) {
      var keyEvent = new KeyEvent.wrap(e);
      if (keyEvent.keyCode == KeyCode.ENTER) {
        if(input.value == "") {
          name = "untitled";
        }
        else {
          name = input.value;
        }
        complete();
      }
    });
  }

  void _deleteWorkspace(CommanderMessage m) {
    var modal = new UpDroidDeleteWorkspaceModal();
    ButtonElement commit = modal.passRefs();

    void complete() {
      String activeNum;
      for(var explorer in _explorersDiv.children) {
        if(explorer.id != 'recycle' && !explorer.classes.contains('control-buttons')) {
          if(!explorer.classes.contains('hidden')) {
            activeNum = explorer.dataset['num'];
            // remove dom element
            explorer.remove();
            // remove corresponding list item
            querySelector("#exp-li-$activeNum").remove();
            // remove from list of updroid explorers
            var toRemove;
            for(var upExp in _columns[0]) {
              if (int.parse(activeNum) == upExp.expNum) {
                toRemove = upExp;
              }
            }
            _columns[0].remove(toRemove);
            toRemove.destroyRecycleListeners();
            toRemove.destroyEditorListeners();
            // Destroy UpDroid Explorer
            _destroyExplorer(toRemove);
          }
        }
      }
      // make first explorer visible
      if (_explorersDiv.children.length > 2) {
        _explorersDiv.children[0].classes.remove('hidden');
      }
      // Destroy cmdr explorer
      _mailbox.ws.send('[[CLOSE_EXPLORER]]' + activeNum);
    }

    commit.onClick.listen((e) {
      complete();
    });
  }

  void _getClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');

  void _registerMailbox() {
    _mailbox.registerCommanderEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerCommanderEvent('OPEN_TAB', _openTabFromButton);
    _mailbox.registerCommanderEvent('GIT_PASSWORD', _gitPassword);
    _mailbox.registerCommanderEvent('SERVER_DISCONNECT', _alertDisconnect);
    _mailbox.registerCommanderEvent('ADD_WORKSPACE', _addWorkspace);
    _mailbox.registerCommanderEvent('DELETE_WORKSPACE', _deleteWorkspace);

    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'GET_CONFIG', _getClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _initializeClient);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers(String config) {
    _newButtonLeft.onClick.listen((e) {
      e.preventDefault();
      if (_columns[1].length >= 4) return;

      new UpDroidOpenTabModal(1, _cs);
    });

    _newButtonRight.onClick.listen((e) {
      e.preventDefault();
      if (_columns[2].length >= 4) return;

      new UpDroidOpenTabModal(2, _cs);
    });
  }

  void _destroyExplorer(UpDroidExplorer explorer) {
    explorer.closed = true;
    explorer = null;
  }
}
