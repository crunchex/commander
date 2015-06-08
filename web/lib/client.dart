library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'tabs/tab_controller.dart';
import 'tabs/teleop.dart';
import 'explorer/explorer.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'mailbox.dart';

class UpDroidClient {
  StreamController<CommanderMessage> _cs;

  List<UpDroidExplorer> _explorers;
  List<List<TabController>> _tabs;
  String _config;

  AnchorElement _addWorkspace;
  AnchorElement _deleteWorkspace;
  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;
  ButtonElement _cleanButton;
  ButtonElement _buildButton;
  ButtonElement _controlButton;
  ButtonElement _runButton;
  ButtonElement _uploadButton;
  DivElement _explorersDiv;

  bool disconnectAlert = false;

  Mailbox _mailbox;
  bool _controlButtonEnabled;

  UpDroidClient() {
    _config = _getConfig();

    _explorers = [];
    _tabs = [[], []];

    _explorersDiv = querySelector('#exp-container');
    _addWorkspace = querySelector('#add-ws');
    _deleteWorkspace = querySelector('#delete-ws');
    _newButtonLeft = querySelector('#column-0-new');
    _newButtonRight = querySelector('#column-1-new');
    _cleanButton = querySelector('#clean-button');
    _controlButton = querySelector('#control-toggle');
    _buildButton = querySelector('#build-button');
    _runButton = querySelector('#run-button');
    _uploadButton = querySelector('#upload');

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
      [{'id': 1, 'class': 'UpDroidEditor'}],
      [{'id': 1, 'class': 'UpDroidConsole'}]
    ];

//    List listConfig = [[],[]];

    return JSON.encode(listConfig);
  }

  int _getAvailableId(String className) {
    List ids = [];

    // Add all used ids for [className] to ids.
    for (int i = 0; i <= 1; i++) {
      _tabs[i].forEach((tab) {

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
  /// TODO: create an abstract class [UpDroidTab] that all others implement.
  /// TODO: call a generic UpDroidTab constructor instead of ifs or switches.
  void _initializeTabs(String strConfig, List explorerPaths) {
    List config = JSON.decode(strConfig);
    int i = 1;
    if(explorerPaths.isEmpty) {
      _addWorkspace.click();
    }
    for (var name in explorerPaths) {
      _openExplorer(i, name);
      i += 1;
    }

    for (int i = 0; i < config.length; i++) {
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

  void _openExplorer(int id, name) {
    if (_explorers.isNotEmpty) {
      for (var explorer in _explorers) {
        explorer.hideExplorer();
      }
    }
    _explorers.add(new UpDroidExplorer(_cs, id, name));
  }

  void _openTab (int column, int id, String className) {
    if (_tabs[column].length >= 4) return;

    if (_tabs[column].isNotEmpty) {
      for (var tab in _tabs[column]) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _tabs[column].add(new UpDroidEditor(id, column, _cs));
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _tabs[column].add(new UpDroidCamera(id, column));
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _tabs[column].add(new UpDroidTeleop(id, column));
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _tabs[column].add(new UpDroidConsole(id, column, _cs));
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
    for (int i = 0; i <= 1; i++) {
      for (int j = 0; j < _tabs[i].length; j++) {
        if (_tabs[i][j].tabType == type && _tabs[i][j].id == num) {
          _tabs[i].removeAt(j);
          col = i;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    for (int j = 0; j < _tabs[col].length; j++) {
      _tabs[col][j].makeInactive();
    }
    _tabs[col].last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + id);
  }

  void _openTabFromButton(CommanderMessage m) {
    List idList = m.body.split('_');
    int column = int.parse(idList[0]);
    String className = idList[1];

    int id = _getAvailableId(className);
    _openTab(column, id, className);
  }

  void _gitPassword(CommanderMessage m) {
    _mailbox.ws.send('[[GIT_PUSH]]' + '${_explorers.first.currentSelectedPath}++${m.body}');
  }

  void _workspaceClean(CommanderMessage m) {
    _cleanButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
    _cleanButton.children.first.classes.add('glyphicons-cleaning');
  }

  void _workspaceBuild(CommanderMessage m) {
    // Success else failure.
    if (m.body == '') {
      _runButton.classes.remove('control-button-disabled');
      _controlButton.classes.remove('control-button-disabled');
      _controlButtonEnabled = true;
    } else {
      new UpDroidBuildResultsModal(m.body);
    }

    _buildButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
    _buildButton.children.first.classes.add('glyphicons-classic-hammer');
  }

  void _sendClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');
  void _serverReady(UpDroidMessage um) => _initializeTabs(_config, JSON.decode(um.body));

  void _registerMailbox() {
    _mailbox.registerCommanderEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerCommanderEvent('OPEN_TAB', _openTabFromButton);
    _mailbox.registerCommanderEvent('GIT_PASSWORD', _gitPassword);
    _mailbox.registerCommanderEvent('WORKSPACE_CLEAN', _workspaceClean);
    _mailbox.registerCommanderEvent('WORKSPACE_BUILD', _workspaceBuild);
    _mailbox.registerCommanderEvent('SERVER_DISCONNECT', _alertDisconnect);

    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'CLIENT_CONFIG', _sendClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLIENT_SERVER_READY', _serverReady);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers(String config) {

    _addWorkspace.onClick.listen((e) {
      String name;

      void complete() {
        var newNum = 1;
        var nums = [];
        var names = [];
        for (var explorer in _explorers) {
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
        _openExplorer(newNum, name);
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

    });

    // TODO: need to find better way for client to track active explorer
    _deleteWorkspace.onClick.listen((e) {
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
              for(var upExp in _explorers) {
                if (int.parse(activeNum) == upExp.expNum) {
                  toRemove = upExp;
                }
              }
              _explorers.remove(toRemove);
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
    });

    _newButtonLeft.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[0].length >= 4) return;

      new UpDroidOpenTabModal(0, _cs);
    });

    _newButtonRight.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[1].length >= 4) return;

      new UpDroidOpenTabModal(1, _cs);
    });

    _cleanButton.onClick.listen((e) {
      _cleanButton.children.first.classes.remove('glyphicons-cleaning');
      _cleanButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);

      _cs.add(new CommanderMessage('UPDROIDEXPLORER', 'WORKSPACE_CLEAN'));

      _controlButton.classes.add('control-button-disabled');
      _runButton.classes.add('control-button-disabled');
      _controlButtonEnabled = false;
    });

    _buildButton.onClick.listen((e) {
      _buildButton.children.first.classes.remove('glyphicons-classic-hammer');
      _buildButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);

      _cs.add(new CommanderMessage('UPDROIDEXPLORER', 'WORKSPACE_BUILD'));
    });

    _controlButton.onClick.listen((e) {
      if (!_controlButtonEnabled) return;
      _cs.add(new CommanderMessage('UPDROIDEXPLORER', 'CATKIN_NODE_LIST'));
    });

    _runButton.onClick.listen((e) {
      if (!_controlButtonEnabled) return;
      _cs.add(new CommanderMessage('UPDROIDEXPLORER', 'RUN_NODE'));
    });

    _uploadButton.onClick.listen((e) {
      new UpDroidGitPassModal(_cs);
    });
  }

  void _destroyExplorer(UpDroidExplorer explorer) {
    explorer.closed = true;
    explorer = null;
  }
}
