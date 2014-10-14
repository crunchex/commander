part of client;

void registerConsoleEventHandlers(WebSocket ws) {
  
  TextInputElement input = querySelector('#input');
  ParagraphElement output = querySelector('#output');
  
  ws.onOpen.listen((Event e) {
    outputMessage(output, 'Connected to server');
  });
  
  input.onChange.listen((Event e) {
    ws.send(input.value.trim());
    input.value = "";
  });

  ws.onMessage.listen((MessageEvent e) {
    String data = e.data.toString();
    if (data.startsWith(new RegExp('DIRECTORY_LIST'))) {
      updateFileExplorer(data);
    } else {
      outputMessage(output, e.data);
    }
  });

  ws.onClose.listen((Event e) {
    outputMessage(output, 'Connection to server lost...');
  });
}

void outputMessage(Element e, String message) {
  print(message);
  e.appendText(message);
  e.appendHtml('<br/>');

  //Make sure we 'autoscroll' the new messages
  e.scrollTop = e.scrollHeight;
}