library client;

import 'dart:html';
import 'package:bootjack/bootjack.dart';
import 'package:ace/ace.dart';
import 'package:ace/proxy.dart';
import 'package:dnd/dnd.dart';

part 'explorer.dart';
part 'editor.dart';
part 'console.dart';

void main() {
  print("Client has started!");

  setUpBootstrap();
  
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  
  UpDroidEditor ed = new UpDroidEditor(ws, 1);
  
  UpDroidExplorer fe = new UpDroidExplorer(ws, ed);
  UpDroidConsole cs = new UpDroidConsole(ws);
  
  registerWebSocketEventHandlers(ws, ed, fe, cs);
}

void setUpBootstrap() {
  Tab.use();
  Button.use();
  Dropdown.use();
  Modal.use();
  //Transition.use();
}

void registerWebSocketEventHandlers(WebSocket ws, UpDroidEditor ed, UpDroidExplorer fe, UpDroidConsole cs) {
  ws.onOpen.listen((Event e) {
      cs.updateOutputField('Connected to updroid!');
      ws.send('[[EXPLORER_DIRECTORY_PATH]]');
    });

  ws.onMessage.listen((MessageEvent e) {
    CommanderMessage cm = new CommanderMessage(e.data);
    
    switch (cm.header()) {
      case 'EXPLORER_DIRECTORY_LIST':
        fe.syncExplorer(cm.body());
        break;
        
      case 'EXPLORER_DIRECTORY_PATH':
        fe.absolutePathPrefix = cm.body();
        ed.absolutePathPrefix = cm.body();
        break;
        
      case 'EDITOR_FILE_TEXT':
        ed.openText(cm.body());
        break;
        
      case 'CONSOLE_COMMAND':
        cs.updateOutputField(e.data);
        break;
        
      default:
        print('Message received without commander header');
    }
  });

  ws.onClose.listen((Event e) {
    cs.updateOutputField('Disconnected from updroid...');
  });
}

class CommanderMessage {
  final String s;
  
  CommanderMessage(this.s);
  
  String header() {
    var header = new RegExp(r'^\[\[[A-Z_]+\]\]').firstMatch(s)[0];
    return header.replaceAll(new RegExp(r'\[\[|\]\]'), '');
  }
  
  String body() => s.replaceFirst(new RegExp(r'^\[\[[A-Z_]+\]\]'), '');
}