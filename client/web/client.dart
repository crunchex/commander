library client;

import 'dart:html';
import 'package:bootjack/bootjack.dart';
import 'package:ace/ace.dart' as ace;
import 'package:ace/proxy.dart';

part 'console.dart';
part 'editor.dart';

void main() {
  print("Client has started!");

  setUpBootstrap();
  setUpWebSocket();
  setUpEditor();
}

void setUpBootstrap() {
  Tab.use();
}