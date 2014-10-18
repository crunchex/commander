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
}

void registerWebSocketEventHandlers(WebSocket ws, UpDroidEditor ed, UpDroidExplorer fe, UpDroidConsole cs) {
  ws.onOpen.listen((Event e) {
      cs.updateOutputField('Connected to updroid!');
      ws.send('REQUEST_DIRECTORY_PATH');
    });

  ws.onMessage.listen((MessageEvent e) {
    String data = e.data.toString();
    if (data.startsWith('RESPONSE_DIRECTORY_LIST')) {
      fe.updateFileExplorer(data.replaceFirst('RESPONSE_DIRECTORY_LIST', ''));
    } else if (data.startsWith('RESPONSE_DIRECTORY_PATH')) {
      fe.absolutePathPrefix = data.replaceFirst('RESPONSE_DIRECTORY_PATH', '');
    } else if (data.startsWith('RESPONSE_FILE_TEXT')) {
      ed.openText(data.replaceFirst('RESPONSE_FILE_TEXT', ''));
    } else {
      cs.updateOutputField(e.data);
    }
  });

  ws.onClose.listen((Event e) {
    cs.updateOutputField('Disconnected from updroid...');
  });
}