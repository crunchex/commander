part of client;

void setUpWebSocket() {
  WebSocket ws = new WebSocket('ws://localhost:8080/ws');
  
  TextInputElement input = querySelector('#input');
  ParagraphElement output = querySelector('#output');
  ButtonElement refresh = querySelector('#button-refresh');
  
  /* File Explorer Stuff */
  refresh.onClick.listen((e) {
    ws.send('REQUEST FILESYSTEM UPDATES');
  });
  
  /* Console Stuff */
  ws.onOpen.listen((Event e) {
    outputMessage(output, 'Connected to server');
  });

  ws.onMessage.listen((MessageEvent e) {
    String data = e.data.toString();
    if (data.startsWith(new RegExp('DIRECTORY_LIST'))) {
      updateFileExplorer(data);
    } else {
      outputMessage(output, e.data);
    }
  });
  
  input.onChange.listen((Event e) {
    ws.send(input.value.trim());
    input.value = "";
  });

  ws.onClose.listen((Event e) {
    outputMessage(output, 'Connection to server lost...');
  });
}

void updateFileExplorer(String data) {
  String strippedHeader = data.replaceFirst(new RegExp('DIRECTORY_LIST: '), '');
  List<String> files = strippedHeader.split(' ');
  
  DivElement well = querySelector('#well-explorer');
  well.innerHtml = '<p>File Explorer</p><ul>';
  
  for (String file in files) {
    well.appendHtml('<li>${file}</li>');
  }
  well.appendHtml('</ul>');
}

void outputMessage(Element e, String message) {
  print(message);
  e.appendText(message);
  e.appendHtml('<br/>');

  //Make sure we 'autoscroll' the new messages
  e.scrollTop = e.scrollHeight;
}