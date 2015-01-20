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
  
  UpDroidClient() {
    // Create the server-client [WebSocket].
    WebSocket ws = new WebSocket('ws://localhost:8080/ws');
    
    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    StreamController<CommanderMessage> cs = new StreamController<CommanderMessage>.broadcast();

    registerEventHandlers(ws, cs);
  }
  
  /// Initializes the main Commander classes.
  void initializeClasses(String workspacePath, WebSocket ws, StreamController<CommanderMessage> cs) {
    editor = new UpDroidEditor(ws, cs, workspacePath, 1);
    explorer = new UpDroidExplorer(ws, cs, workspacePath);
    console = new UpDroidConsole(ws, cs);
  }
  
  /// Process messages that Console has picked up according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CONSOLE_READY':
        console.updateOutputHandler(m.body);
        break;
        
      default:
        print('Console error: unrecognized message type.');
    }
  }
  
  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void registerEventHandlers(WebSocket ws, StreamController<CommanderMessage> cs) {
    ws.onOpen.listen((um) {
      cs.add(new CommanderMessage('CONSOLE', 'OUTPUT', body: 'Connected to updroid.'));
      ws.send('[[EXPLORER_DIRECTORY_PATH]]');
    });
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_PATH')
        .listen((um) => initializeClasses(um.body, ws, cs));
  
    ws.onClose.listen((event) => cs.add(new CommanderMessage('CONSOLE', 'OUTPUT', body: 'Disconnected from updroid.')));
    
    cs.stream
        .where((m) => m.dest == 'CLIENT')
        .listen((m) => processMessage(m));
  }
}