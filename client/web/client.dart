library client.dart;

import 'dart:html';
import 'package:ace/ace.dart' as ace;
import 'package:ace/proxy.dart';



void main() {
  print("Client has started!");

  setUpEditor();

  TextInputElement input = querySelector('#input');
  ParagraphElement output = querySelector('#output');

  String server = 'ws://localhost:8080/ws';
  WebSocket ws = new WebSocket(server);
  ws.onOpen.listen((Event e) {
    outputMessage(output, 'Connected to server');
  });

  ws.onMessage.listen((MessageEvent e) {
    outputMessage(output, e.data);
  });

  ws.onClose.listen((Event e) {
    outputMessage(output, 'Connection to server lost...');
  });

  input.onChange.listen((Event e) {
    ws.send(input.value.trim());
    input.value = "";
  });
}

void setUpEditor() {
  ace.implementation = ACE_PROXY_IMPLEMENTATION;
  
  ace.Editor editor = ace.edit(querySelector('#editor'));
  editor
      ..theme = new ace.Theme.named(ace.Theme.CHROME)
      ..session.mode = new ace.Mode.named(ace.Mode.DART);
      //..setValue(ace.Mode.DART, -1);
}

void outputMessage(Element e, String message) {
  print(message);
  e.appendText(message);
  e.appendHtml('<br/>');

  //Make sure we 'autoscroll' the new messages
  e.scrollTop = e.scrollHeight;
}
