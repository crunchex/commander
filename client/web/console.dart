part of client;

class UpDroidConsole {
  WebSocket ws;
  
  DivElement inputGroup;
  TextInputElement input;
  ParagraphElement output;
  AnchorElement consoleButton;
  AnchorElement themeButton;
  
  UpDroidConsole(WebSocket ws) {
    this.ws = ws;
    inputGroup = querySelector('#input-group');
    input = querySelector('#input');
    output = querySelector('#output');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
    registerConsoleEventHandlers();
  }

  void registerConsoleEventHandlers() {
    input.onChange.listen((e) {
      ws.send('[[CONSOLE_COMMAND]]' + input.value.trim());
      input.value = "";
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
    
    consoleButton.onClick.listen((e) {
      // This is broken.
      input.focus();
    });
  }

  void updateOutputField(String message) {
    output.appendText('up> ${message}');
    output.appendHtml('<br/>');

    // Make sure we 'autoscroll' the new messages.
    output.scrollTop = output.scrollHeight;
  }
  
  void toggleTheme() {
    if (inputGroup.style.backgroundColor == 'rgb(238, 232, 213)') {
      inputGroup.style.backgroundColor = '#002b36'; // base-green
      input.style.color = '#268bd2';  // blue
      output.style.color = '#93a1a1'; // light-grey
    } else {
      inputGroup.style.backgroundColor = '#eee8d5'; // base-tan
      input.style.color = '#dc322f';  // red
      output.style.color = '#586e75'; // dark-grey
    }
  }
}