library client;

import 'dart:html';
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
  
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  
  UpDroidEditor ed = new UpDroidEditor(ws, 1);
  
  UpDroidExplorer fe = new UpDroidExplorer(ws, ed);
  UpDroidConsole cs = new UpDroidConsole(ws);
  
  registerWebSocketEventHandlers(ws, ed, fe, cs);
}

/// Activates Bootjack features.
void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Modal.use();
  Transition.use();
}

/// Sets up external event handlers for the various Commander classes. These
/// are mostly listening events for [WebSocket] messages.
void registerWebSocketEventHandlers(WebSocket ws, UpDroidEditor ed, UpDroidExplorer fe, UpDroidConsole cs) {
  ws.onOpen.listen((Event e) {
      cs.updateOutputField('Connected to updroid.');
      ws.send('[[EXPLORER_DIRECTORY_PATH]]');
    });

  ws.onMessage.listen((MessageEvent e) {
    UpDroidMessage um = new UpDroidMessage(e.data);
    
    switch (um.header) {
      case 'EXPLORER_DIRECTORY_LIST':
        fe.syncExplorer(um.body);
        break;
        
      case 'EXPLORER_DIRECTORY_PATH':
        fe.absolutePathPrefix = um.body;
        ed.absolutePathPrefix = um.body;
        break;
        
      case 'EDITOR_FILE_TEXT':
        ed.openText(um.body);
        break;
        
      case 'CONSOLE_COMMAND':
        cs.updateOutputField(um.body);
        break;
        
      default:
        print('Message received without commander header');
    }
  });

  ws.onClose.listen((Event e) {
    cs.updateOutputField('Disconnected from updroid.');
  });
}