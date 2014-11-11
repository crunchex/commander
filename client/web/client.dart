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

void main() {
  setUpBootstrap();
  
  // Create the server-client [WebSocket].
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  
  // Create the intra-client message stream.
  // The classes use this to communicate with each other.
  StreamController<CommanderMessage> cs = new StreamController<CommanderMessage>.broadcast();

  registerEventHandlers(ws, cs);
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  //Modal.use();
  Transition.use();
  
  Popover.wire(querySelector('#console-help.dropdown-toggle'));
}

/// Initializes the main Commander classes.
void initializeClasses(String workspacePath, WebSocket ws, StreamController<CommanderMessage> cs) {
  UpDroidEditor editor = new UpDroidEditor(ws, cs, workspacePath, 1);
  UpDroidExplorer explorer = new UpDroidExplorer(ws, cs, workspacePath);
  UpDroidConsole console = new UpDroidConsole(ws, cs);
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
}