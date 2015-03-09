library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
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

class UpDroidClient {
  UpDroidEditor editor;
  UpDroidExplorer explorer;
  UpDroidConsole console;

  WebSocket ws;
  StreamController<CommanderMessage> cs;

  String status;
  bool encounteredError;

  UpDroidClient() {
    this.status = 'DISCONNECTED';
    this.encounteredError = false;

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    this.cs = new StreamController<CommanderMessage>.broadcast();

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.

    //  Uncomment for build
    //  initWebSocket('ws://' + window.location.host + '/ws');
    initWebSocket('ws://localhost:12060/ws');

    registerEventHandlers(ws, cs);
    initializeClasses(ws, cs);
  }

  /// Initializes the main Commander classes.
  void initializeClasses(WebSocket ws, StreamController<CommanderMessage> cs) {
    editor = new UpDroidEditor(ws, cs);
    explorer = new UpDroidExplorer(ws, cs);
    console = new UpDroidConsole(cs);
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    // The classes still need to be informed of connection status individually
    // because they are initialized after first 'CONNECTED' broadcast.
    // Subsequent connection notifications can be grouped together.
    switch (m.type) {
      case 'CONSOLE_READY':
        //cs.add(new CommanderMessage('CONSOLE', status));
        break;

      case 'EXPLORER_READY':
        //cs.add(new CommanderMessage('EXPLORER', status));
        break;

      case 'EDITOR_READY':
        break;

      default:
        print('Client error: unrecognized message type: ' + m.type);
    }
  }

  void initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    ws = new WebSocket(url);

    ws.onOpen.listen((e) {
      status = 'CONNECTED';
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
  void registerEventHandlers(WebSocket ws, StreamController<CommanderMessage> cs) {
    cs.stream
        .where((m) => m.dest == 'CLIENT')
        .listen((m) => processMessage(m));
  }
}