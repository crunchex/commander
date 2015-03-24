library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'editor.dart';
import 'console.dart';
import 'explorer.dart';
import 'camera.dart';
import 'lib/updroid_message.dart';

class UpDroidClient {
  // TODO: find syntax to make this not such a long line.
  static const String defaultConfig = '{"0":["UpDroidExplorer"],"1":["UpDroidEditor", "UpDroidEditor", "UpDroidCamera"],"2":["UpDroidConsole","UpDroidConsole","UpDroidConsole","UpDroidConsole"]}';

  WebSocket ws;
  StreamController<CommanderMessage> cs;

  String _config;
  List<UpDroidExplorer> _explorers = [];
  List<UpDroidEditor> _editors = [];
  List<UpDroidConsole> _consoles = [];
  List<UpDroidCamera> _cameras = [];

  String status;
  bool encounteredError;

  UpDroidClient() {
    this.status = 'DISCONNECTED';
    this.encounteredError = false;

    _config = _getConfig();

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    this.cs = new StreamController<CommanderMessage>.broadcast();

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host;
    url = url.split(':')[0];
    initWebSocket('ws://' + url + ':12060/server/1');

    registerEventHandlers(ws, cs);
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
      ws.send('[[CLIENT_CONFIG]]' + _config);
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
  void registerEventHandlers(WebSocket ws, StreamController<CommanderMessage> cs) {
    cs.stream
        .where((m) => m.dest == 'CLIENT')
        .listen((m) => processMessage(m));
  }

  /// Returns a [Map] of all the tabs that [UpDroidClient] needs to spawn,
  /// where key = side|left|right| and value = a list of UpDroidTab.className.
  /// TODO: this function should retrieve information from some defaults or
  /// a user account settings save.
  String _getConfig([String config]) {
    if (config == null) {
      config = UpDroidClient.defaultConfig;
    }

    return config;
  }

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: create an abstract class [UpDroidTab] that all others implement.
  /// TODO: call a generic UpDroidTab constructor instead of ifs or switches.
  void _initializeTabs() {
    Map tabs = JSON.decode(_config);

    for (String column in tabs.keys) {
      for (String guiName in tabs[column]) {
        if (guiName == UpDroidExplorer.className) {
          _explorers.add(new UpDroidExplorer(cs));
        } else if (guiName == UpDroidEditor.className) {
          if (_editors.isEmpty) {
            _editors.add(new UpDroidEditor(_editors.length + 1, int.parse(column), cs, active: true));
          } else {
            _editors.add(new UpDroidEditor(_editors.length + 1, int.parse(column), cs));
          }
        } else if (guiName == UpDroidCamera.className) {
          _cameras.add(new UpDroidCamera(_cameras.length + 1));
        } else if (guiName == UpDroidConsole.className) {
          _consoles.add(new UpDroidConsole(_consoles.length + 1, cs));
        }
      }
    }
  }

  void _closeTab(String id) {
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    switch (type) {
      case 'EDITOR':
        _editors.removeAt(num - 1);
        break;
    }

    ws.send('[[CLOSE_TAB]]' + id);
  }
}