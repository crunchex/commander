library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'tab.dart';
import 'modal.dart';
import 'editor.dart';
import 'console.dart';
import 'explorer.dart';
import 'camera.dart';
import 'lib/updroid_message.dart';

class UpDroidClient {
  // TODO: find syntax to make this not such a long line.

  WebSocket ws;
  StreamController<CommanderMessage> cs;

  List<List> _tabs;

  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;

  String status;
  bool encounteredError;
  String currentPath;

  UpDroidClient() {
    this.status = 'DISCONNECTED';
    this.encounteredError = false;

    _tabs = [[], [], []];

    _newButtonLeft = querySelector('#column-1-new');
    _newButtonRight = querySelector('#column-2-new');

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

    ws.onMessage.transform(updroidTransformer)
      .where((um) => um.header == 'CLIENT_SERVER_READY')
      .listen((um) => _initializeTabs());


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

    _newButtonLeft.onClick.listen((e) {
      if (_tabs[1].length >= 4) return;

      String classType = 'UpDroidConsole';
      int id = _getAvailableId(classType);
      _openTab(1, id, classType);
    });

    _newButtonRight.onClick.listen((e) {
      if (_tabs[2].length >= 4) return;

      String classType = 'UpDroidEditor';
      int id = _getAvailableId(classType);
      _openTab(2, id, classType);
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
        {'id': 1, 'class': 'UpDroidCamera'},
        {'id': 1, 'class': 'UpDroidEditor'},
        {'id': 2, 'class': 'UpDroidEditor'}],
      [
        {'id': 1, 'class': 'UpDroidConsole'},
        {'id': 2, 'class': 'UpDroidConsole'},
        {'id': 3, 'class': 'UpDroidConsole'}]
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

    int i = 0;
    for (List column in config) {
      for (Map tab in config[i]) {
        _openTab(i + 1, tab['id'], tab['class']);
      }
      i++;
    }
  }

  void _openTab(int column, int id, String className) {
    ws.send('[[OPEN_TAB]]' + '$column-$id-$className');

    if (_tabs[column].isNotEmpty) _tabs[column].last.makeTabInactive();

    if (className == UpDroidEditor.className) {
      _tabs[column].add(new UpDroidEditor(id, column, cs, active: true));
    } else if (className == UpDroidCamera.className) {
      _tabs[column].add(new UpDroidCamera(id, column, cs, active: true));
    } else if (className == UpDroidConsole.className) {
      _tabs[column].add(new UpDroidConsole(id, column, cs, active: true));
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