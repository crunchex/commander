library client;

import 'dart:html';
import 'dart:async';
import 'package:bootjack/bootjack.dart';
import 'package:ace/ace.dart';
import 'package:ace/proxy.dart';
import 'package:dnd/dnd.dart';
import 'lib/updroid_message.dart';

part 'explorer.dart';
part 'editor.dart';
part 'console.dart';

const String EXPLORER_DIRECTORY_PATH = '[[EXPLORER_DIRECTORY_PATH]]';

void main() {
  setUpBootstrap();
  
  // Create the server-client [WebSocket].
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  
  // Create the intra-client message stream.
  // The classes use this to communicate with each other.
  StreamController<String> cs = new StreamController<String>();

  registerWebSocketEventHandlers(ws, cs);
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Modal.use();
  Transition.use();
}

/// Initializes the main Commander classes.
void initializeClasses(String raw, WebSocket ws, StreamController<String> cs) {
  UpDroidMessage um = new UpDroidMessage(raw);
  
  UpDroidEditor editor = new UpDroidEditor(ws, um.body, 1);
  UpDroidExplorer explorer = new UpDroidExplorer(ws, um.body, editor);
  UpDroidConsole console = new UpDroidConsole(ws, cs);
}

/// Sets up external event handlers for the various Commander classes. These
/// are mostly listening events for [WebSocket] messages.
void registerWebSocketEventHandlers(WebSocket ws, StreamController<String> cs) {
  ws.onOpen.listen((Event e) {
      cs.add('[[CONSOLE_OUTPUT]]' + 'Connected to updroid.');
      ws.send(EXPLORER_DIRECTORY_PATH);
  });
  
  ws.onMessage
      .where((value) => value.data.startsWith(EXPLORER_DIRECTORY_PATH))
      .listen((value) => initializeClasses(value.data, ws, cs));

  ws.onClose.listen((Event e) {
    cs.add('[[CONSOLE_OUTPUT]]' + 'Disconnected from updroid.');
  });
}