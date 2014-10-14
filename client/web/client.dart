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
  
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  registerConsoleEventHandlers(ws);
  registerExplorerEventHandlers(ws);
  setUpEditor();
}

void setUpBootstrap() {
  Tab.use();
}