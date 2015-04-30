library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'tab.dart';
import 'editor.dart';
import 'console.dart';
import 'explorer.dart';
import 'camera.dart';
import 'modal.dart';
import 'lib/updroid_message.dart';

class UpDroidClient {
  // TODO: find syntax to make this not such a long line.

  WebSocket ws;
  StreamController<CommanderMessage> cs;

  List<List> _tabs;

  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;
  ButtonElement _cleanButton;
  ButtonElement _buildButton;
  ButtonElement _runButton;
  ButtonElement _uploadButton;

  String status;
  bool encounteredError;
  String currentPath;
  bool _runButtonEnabled;

  ElementStream chooseEditor;
  ElementStream chooseConsole;
  ElementStream chooseVideo;

  UpDroidClient() {
    this.status = 'DISCONNECTED';
    this.encounteredError = false;

    _tabs = [[], [], []];

    _newButtonLeft = querySelector('#column-1-new');
    _newButtonRight = querySelector('#column-2-new');
    _cleanButton = querySelector('#clean-button');
    _buildButton = querySelector('#build-button');
    _runButton = querySelector('#run-button');
    _uploadButton = querySelector('#upload');

    _runButtonEnabled = true;

    String config = _getConfig();

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    this.cs = new StreamController<CommanderMessage>.broadcast();

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host;
    url = url.split(':')[0];
    initWebSocket('ws://' + url + ':12060/server/1');

    registerEventHandlers(ws, cs, config);
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CLOSE_TAB':
        _closeTab(m.body);
        break;

      case 'OPEN_TAB':
        List idList = m.body.split('_');
        int column = int.parse(idList[0]);
        String className = idList[1];

        int id = _getAvailableId(className);
        _openTab(column, id, className);
        break;

      case 'GIT_PASSWORD':
        // TODO: need a more reliable handle to Explorer.
        ws.send('[[GIT_PUSH]]' + '${_tabs[0].first.currentSelectedPath}++${m.body}');
        break;

      default:
        print('Client warning: received unrecognized message type ${m.type}');
    }
  }

  void initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    ws = new WebSocket(url);

    ws.onOpen.listen((e) {
      status = 'CONNECTED';
      ws.send('[[CLIENT_CONFIG]]');
      cs.add(new CommanderMessage('ALL', status));
    });

    ws.onClose.listen((e) {
      status = 'DISCONNECTED';
      cs.add(new CommanderMessage('ALL', status));
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });

    ws.onError.listen((e) {
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void registerEventHandlers(WebSocket ws, StreamController<CommanderMessage> cs, String config) {
    cs.stream
        .where((m) => m.dest == 'CLIENT')
        .listen((m) => processMessage(m));

    ws.onMessage.transform(updroidTransformer)
      .where((um) => um.header == 'CLIENT_SERVER_READY')
      .listen((um) => _initializeTabs(config));

    ws.onMessage.transform(updroidTransformer)
      .where((um) => um.header == 'WORKSPACE_CLEAN_DONE')
      .listen((um) {
      print('clean done');
        _cleanButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
        _cleanButton.children.first.classes.add('glyphicons-cleaning');
      });

    ws.onMessage.transform(updroidTransformer)
      .where((um) => um.header == 'BUILD_RESULT')
      .listen((um) {
        // Success.
        if (um.body == '') {
          _runButton.classes.remove('control-button-disabled');
          _runButtonEnabled = true;
        } else {
          new UpDroidBuildResultsModal(um.body);
        }

        _cleanButton.children.first.classes.removeAll(['glyphicons-refresh', 'glyph-progress']);
        _cleanButton.children.first.classes.add('glyphicons-classic-hammer');
      });

    ws.onMessage.transform(updroidTransformer)
      .where((um) => um.header == 'CATKIN_NODE_LIST')
      .listen((um) {
        new UpDroidRunNodeModal(JSON.decode(um.body), ws);
      });

    _newButtonLeft.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[1].length >= 4) return;

      new UpDroidOpenTabModal(1, cs);
    });

    _newButtonRight.onClick.listen((e) {
      e.preventDefault();
      if (_tabs[2].length >= 4) return;

      new UpDroidOpenTabModal(2, cs);
    });

    _cleanButton.onClick.listen((e) {
      _cleanButton.children.first.classes.remove('glyphicons-cleaning');
      _cleanButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);

      ws.send('[[WORKSPACE_CLEAN]]');

      _runButton.classes.add('control-button-disabled');
      _runButtonEnabled = false;
    });

    _buildButton.onClick.listen((e) {
      _cleanButton.children.first.classes.remove('glyphicons-classic-hammer');
      _cleanButton.children.first.classes.addAll(['glyphicons-refresh', 'glyph-progress']);
      ws.send('[[WORKSPACE_BUILD]]');
    });

    _runButton.onClick.listen((e) {
      if (!_runButtonEnabled) return;
      ws.send('[[CATKIN_NODE_LIST]]');
    });

    _uploadButton.onClick.listen((e) {
      new UpDroidGitPassModal(cs);
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
      [
        {'id': 1, 'class': 'UpDroidEditor'}],
      [
        {'id': 1, 'class': 'UpDroidConsole'}]
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
      if (!ids.contains(id))
        break;
    }

    return id;
  }

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: create an abstract class [UpDroidTab] that all others implement.
  /// TODO: call a generic UpDroidTab constructor instead of ifs or switches.
  void _initializeTabs(String strConfig) {
    List config = JSON.decode(strConfig);

    _tabs[0].add(new UpDroidExplorer(cs));

    for (int i = 0; i < config.length; i++) {
      for (Map tab in config[i]) {
        _openTab(i + 1, tab['id'], tab['class']);
      }
    }
  }

  void _openTab(int column, int id, String className) {
    if (_tabs[column].isNotEmpty) {
      for (var tab in _tabs[column]) {
        tab.makeTabInactive();
      }
    }

    if (className == UpDroidEditor.className) {
      _tabs[column].add(new UpDroidEditor(id, column, cs, active: true));
      ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
    } else if (className == UpDroidCamera.className) {
      _tabs[column].add(new UpDroidCamera(id, column, cs, active: true));
      ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
    } else if (className == UpDroidConsole.className) {
      UpDroidConsole console = new UpDroidConsole(id, column, cs, active: true);
      _tabs[column].add(console);
      // TODO: initial size should not be hardcoded.
      ws.send('[[OPEN_TAB]]' + '$column-$id-$className-25-80');
    }
  }

  void _closeTab(String id) {
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Add all used ids for [className] to ids.
    for (int i = 1; i <= 2; i++) {
      for (int j = 0; j < _tabs[i].length; j++) {
        if (_tabs[i][j].type == type && _tabs[i][j].num == num) {
          _tabs[i].removeAt(j);
          _tabs[i].last.makeTabActive();
        }
      }
    }

    ws.send('[[CLOSE_TAB]]' + id);
  }
}