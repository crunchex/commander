library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'dart:isolate';

import 'explorer/explorer.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'updroid_message.dart';
import 'mailbox.dart';

class UpDroidClient {
  StreamController<CommanderMessage> _cs;

  List<List> _tabs;
  String _config;

  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;
  ButtonElement _cleanButton;
  ButtonElement _buildButton;
  ButtonElement _runButton;
  ButtonElement _uploadButton;

  Mailbox _mailbox;

  String status;
  bool encounteredError;
  String currentPath;
  bool _runButtonEnabled;

  UpDroidClient() {
    _config = _getConfig();

    _tabs = [[], [], []];

    _newButtonLeft = querySelector('#column-1-new');
    _newButtonRight = querySelector('#column-2-new');
    _cleanButton = querySelector('#clean-button');
    _buildButton = querySelector('#build-button');
    _runButton = querySelector('#run-button');
    _uploadButton = querySelector('#upload');

    _runButtonEnabled = true;

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    _cs = new StreamController<CommanderMessage>.broadcast();
    _mailbox = new Mailbox('updroidclient', 1, _cs);

    _registerMailbox();
    registerEventHandlers(_getConfig());

    AnchorElement feedbackButton = querySelector('#feedback-button');
    pulseFeedback(feedbackButton);
  }

  void pulseFeedback(AnchorElement feedbackButton) {
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

    return JSON.encode(listConfig);
  }

  int _getAvailableId(String className) {
    List ids = [];

    // Add all used ids for [className] to ids.
    for (int i = 1; i <= 2; i++) {
      _tabs[i].forEach((tab) {
        if (tab.type == className) ids.add(tab.num);
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
    for (var name in explorerPaths) {
      _openExplorer(i, name);
      i += 1;
    }

    for (int i = 0; i < config.length; i++) {
      for (Map tab in config[i]) {
        _openTab(i + 1, tab['id'], tab['class']);
      }
    }
  }

  void _openExplorer(int id, name) {
    if (_tabs[0].isNotEmpty) {
      for (var explorer in _tabs[0]) {
        explorer.hideExplorer();
      }
    }
    _tabs[0].add(new UpDroidExplorer(_cs, id, name));
  }

  Future _openTab (int column, int id, String className) async {
    if (_tabs[column].isNotEmpty) {
      for (var tab in _tabs[column]) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _tabs[column].add(new UpDroidEditor(id, column, _cs, active: true));
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
    } else if (className == 'UpDroidCamera') {
      _tabs[column].add(new UpDroidCamera(id, column, _cs, active: true));
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
    } else if (className == 'UpDroidConsole') {
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _tabs[column].add(new UpDroidConsole(id, column, _cs, active: true));
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className-25-80');
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void _closeTab(CommanderMessage m) {
    String id = m.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Add all used ids for [className] to ids.
    for (int i = 1; i <= 2; i++) {
      for (int j = 0; j < _tabs[i].length; j++) {
        if (_tabs[i][j].type == type && _tabs[i][j].num == num) {
          _tabs[i].removeAt(j);
          _tabs[i].last.makeActive();
        }
      }
    }

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
    _mailbox.ws.send('[[GIT_PUSH]]' + '${_tabs[0].first.currentSelectedPath}++${m.body}');
  }

  void _workspaceClean(CommanderMessage m) {
    _cleanButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
    _cleanButton.children.first.classes.add('glyphicons-cleaning');
  }

  void _workspaceBuild(CommanderMessage m) {
    // Success else failure.
    if (m.body == '') {
      _runButton.classes.remove('control-button-disabled');
      _runButtonEnabled = true;
    } else {
      new UpDroidBuildResultsModal(m.body);
    }

    _buildButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
    _buildButton.children.first.classes.add('glyphicons-classic-hammer');
  }

  void _sendClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');
  void _serverReady(UpDroidMessage um) => _initializeTabs(_config, JSON.decode(um.body));
  void _nodeList(UpDroidMessage um) { new UpDroidRunNodeModal(JSON.decode(um.body), _mailbox.ws); }

  void _registerMailbox() {
    _mailbox.registerCommanderEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerCommanderEvent('OPEN_TAB', _openTabFromButton);
    _mailbox.registerCommanderEvent('GIT_PASSWORD', _gitPassword);
    _mailbox.registerCommanderEvent('WORKSPACE_CLEAN', _workspaceClean);
    _mailbox.registerCommanderEvent('WORKSPACE_BUILD', _workspaceBuild);

    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'CLIENT_CONFIG', _sendClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLIENT_SERVER_READY', _serverReady);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CATKIN_NODE_LIST', _nodeList);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void registerEventHandlers(String config) {
    _newButtonLeft.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[1].length >= 4) return;

      new UpDroidOpenTabModal(1, _cs);
    });

    _newButtonRight.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[2].length >= 4) return;

      new UpDroidOpenTabModal(2, _cs);
    });

    _cleanButton.onClick.listen((e) {
      _cleanButton.children.first.classes.remove('glyphicons-cleaning');
      _cleanButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);

      _cs.add(new CommanderMessage('EXPLORER', 'WORKSPACE_CLEAN'));

      _runButton.classes.add('control-button-disabled');
      _runButtonEnabled = false;
    });

    _buildButton.onClick.listen((e) {
      _buildButton.children.first.classes.remove('glyphicons-classic-hammer');
      _buildButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);

      _cs.add(new CommanderMessage('EXPLORER', 'WORKSPACE_BUILD'));
    });

    _runButton.onClick.listen((e) {
      if (!_runButtonEnabled) return;
      _mailbox.ws.send('[[CATKIN_NODE_LIST]]');
    });

    _uploadButton.onClick.listen((e) {
      new UpDroidGitPassModal(_cs);
    });
  }
}
