part of client;

class Console {
  WebSocket ws;
  TextInputElement input;
  ParagraphElement output;
  
  Console(WebSocket ws) {
    this.ws = ws;
    this.input = querySelector('#input');
    this.output = querySelector('#output');
    
    registerConsoleEventHandlers();
  }

  void registerConsoleEventHandlers() {
    input.onChange.listen((Event e) {
      print("Input changed!");
      ws.send(input.value.trim());
      input.value = "";
    });
  }

  void updateOutputField(String message) {
    print(message);
    output.appendText(message);
    output.appendHtml('<br/>');

    //Make sure we 'autoscroll' the new messages
    output.scrollTop = output.scrollHeight;
  }
  
}