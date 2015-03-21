library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:js' as js;
import 'package:ace/ace.dart' as ace;
import 'package:bootjack/bootjack.dart';
import 'package:ace/proxy.dart';
import 'package:dnd/dnd.dart';
import 'lib/updroid_message.dart';
import 'lib/explorer_helper.dart';
import 'lib/terminal/terminal.dart';
import "package:path/path.dart" as pathLib;

part 'explorer.dart';
part 'editor.dart';
part 'console.dart';
part 'camera.dart';

class UpDroidClient {
  // TODO: find syntax to make this not such a long line.
  static const String defaultConfig = '{"side":["UpDroidExplorer"],"left":["UpDroidEditor","UpDroidCamera"],"right":["UpDroidConsole","UpDroidConsole","UpDroidConsole","UpDroidConsole"]}';

  WebSocket ws;
  StreamController<CommanderMessage> cs;
  String _config;

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
    initWebSocket('ws://' + url + ':12060/');

    registerEventHandlers(ws, cs);
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    // TODO: evaluate whether this is necesary anymore.
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

    for (String className in tabs['side']) {
      if (className == UpDroidExplorer.className) {
        UpDroidExplorer explorer = new UpDroidExplorer(ws, cs);
      }
    }

    for (String className in tabs['left']) {
      if (className == UpDroidEditor.className) {
        UpDroidEditor editor = new UpDroidEditor(ws, cs);
      } else if (className == UpDroidCamera.className) {
        UpDroidCamera camera = new UpDroidCamera(1);
      }
    }

    int i = 1;
    for (String className in tabs['right']) {
      if (className == UpDroidConsole.className) {
        UpDroidConsole console = new UpDroidConsole(i, cs);
      }
      i++;
    }
  }
}