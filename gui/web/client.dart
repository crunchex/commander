library updroid_client;

import 'dart:html';
import 'dart:async';
import 'package:ace/ace.dart';
import 'package:bootjack/bootjack.dart';
import 'package:ace/proxy.dart';
import 'package:dnd/dnd.dart';
import 'lib/updroid_message.dart';

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
  
  UpDroidClient() {
    this.status = 'DISCONNECTED';

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    this.cs = new StreamController<CommanderMessage>.broadcast();
    
    // Create the server-client [WebSocket].
    this.ws = new WebSocket('ws://localhost:8080/ws');
    
    registerEventHandlers(ws, cs);
    initializeClasses(ws, cs);
  }
  
  /// Initializes the main Commander classes.
  void initializeClasses(WebSocket ws, StreamController<CommanderMessage> cs) {
    editor = new UpDroidEditor(ws, cs);
    explorer = new UpDroidExplorer(ws, cs);
    console = new UpDroidConsole(ws, cs);
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
  
  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void registerEventHandlers(WebSocket ws, StreamController<CommanderMessage> cs) {
    ws.onOpen.listen((um) {
      status = 'CONNECTED';
      cs.add(new CommanderMessage('ALL', status));
    });
  
    ws.onClose.listen((event) {
      status = 'DISCONNECTED';
      cs.add(new CommanderMessage('ALL', status));
    });
    
    cs.stream
        .where((m) => m.dest == 'CLIENT')
        .listen((m) => processMessage(m));
  }
}