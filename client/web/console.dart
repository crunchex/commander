part of client;

/// [UpDroidConsole] functions like a trimmed down terminal that allows the
/// user to pass in white-listed commands to the server and view their output.
/// It is not meant to be a complete terminal emulator like xterm.
class UpDroidConsole {
  static const String CONSOLE_COMMAND = '[[CONSOLE_COMMAND]]';
  
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
  
  /// Toggles between a Solarized dark and light theme.
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
  
  /// Updates the output field based on string messages passed in over the
  /// [WebSocket]. Everything coming in here should be the result of passing a
  /// command over the input, since everything else would have been filtered
  /// out with the message headers.
  void updateOutputHandler(String raw) {
    UpDroidMessage um = new UpDroidMessage(raw);
    output.appendText('up> ${um.body}');
    output.appendHtml('<br/>');

    // Autoscroll the new messages as they come in.
    output.scrollTop = output.scrollHeight;
  }

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage
        .where((value) => value.toString().startsWith(CONSOLE_COMMAND))
        .listen((value) => updateOutputHandler(value.toString()));
    
    input.onChange.listen((e) {
      ws.send('[[CONSOLE_COMMAND]]' + input.value.trim());
      input.value = "";
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
    
    consoleButton.onClick.listen((e) {
      // This is broken. It's supposed to move the cursor focus over to
      // the input field when a user selects the Console tab.
      input.focus();
    });
  }
}