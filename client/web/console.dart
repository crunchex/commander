part of client;

class Console {
  WebSocket ws;
  DivElement inputGroup;
  TextInputElement input;
  ParagraphElement output;
  AnchorElement buttonConsole;
  AnchorElement themeButton;
  
  Console(WebSocket ws) {
    this.ws = ws;
    inputGroup = querySelector('#input-group');
    input = querySelector('#input');
    output = querySelector('#output');
    buttonConsole = querySelector('#button-console');
    themeButton = querySelector('#button-console-theme');
    
    registerConsoleEventHandlers();
  }

  void registerConsoleEventHandlers() {
    input.onChange.listen((e) {
      ws.send(input.value.trim());
      input.value = "";
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
    
    buttonConsole.onClick.listen((e) {
      // This is broken :(
      input.focus();
    });
  }

  void updateOutputField(String message) {
    output.appendText('up> ${message}');
    output.appendHtml('<br/>');

    //Make sure we 'autoscroll' the new messages
    output.scrollTop = output.scrollHeight;
  }
  
  void toggleTheme() {
    if (inputGroup.style.backgroundColor == 'rgb(238, 232, 213)') {
      inputGroup.style.backgroundColor = '#002b36';
      input.style.color = '#268bd2';
      output.style.color = '#93a1a1';
    } else {
      inputGroup.style.backgroundColor = '#eee8d5';
      input.style.color = '#dc322f';
      output.style.color = '#586e75';
    }
  }
}