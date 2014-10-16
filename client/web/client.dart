library client;

import 'dart:html';
import 'package:bootjack/bootjack.dart';
import 'package:ace/ace.dart' as ace;
import 'package:ace/proxy.dart';

part 'explorer.dart';
part 'editor.dart';
part 'console.dart';

void main() {
  print("Client has started!");

  setUpBootstrap();
  Editor e = new Editor();
  
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  FileExplorer fe = new FileExplorer(ws);
  Console cs = new Console(ws);
  
  registerWebSocketEventHandlers(ws, fe, cs);
}

void setUpBootstrap() {
  Tab.use();
  Button.use();
}

void registerWebSocketEventHandlers(WebSocket ws, FileExplorer fe, Console cs) {
  ws.onOpen.listen((Event e) {
      cs.updateOutputField('Connected to updroid!');
      ws.send('REQUEST_DIRECTORY_PATH');
    });

  ws.onMessage.listen((MessageEvent e) {
    String data = e.data.toString();
    if (data.startsWith('RESPONSE_DIRECTORY_LIST')) {
      fe.updateFileExplorer(data.replaceFirst('RESPONSE_DIRECTORY_LIST', ''));
    } else if (data.startsWith('RESPONSE_DIRECTORY_PATH')) {
      fe.directoryPath = data.replaceFirst('RESPONSE_DIRECTORY_PATH', '');
    } else {
      cs.updateOutputField(e.data);
    }
  });

  ws.onClose.listen((Event e) {
    cs.updateOutputField('Disconnected from updroid...');
  });
}