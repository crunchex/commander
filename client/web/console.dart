part of client;

/// [UpDroidConsole] functions like a trimmed down terminal that allows the
/// user to pass in white-listed commands to the server and view their output.
/// It is not meant to be a complete terminal emulator like xterm.
class UpDroidConsole {
  static const String CONSOLE_COMMAND = '[[CONSOLE_COMMAND]]';
  
  WebSocket ws;
  StreamController<CommanderMessage> cs;
  
  DivElement inputGroup;
  TextInputElement input;
  ParagraphElement output;
  AnchorElement consoleButton;
  AnchorElement themeButton;
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    inputGroup = querySelector('#input-group');
    input = querySelector('#input');
    output = querySelector('#output');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
    registerConsoleEventHandlers();
    
    // This is a hack to account for the fact that Console is set up
    // after the initial connection is made and thus, misses the first
    // message.
    updateOutputHandler('Connected to updroid.');
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
  
  /// Process messages that Console has picked up according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'OUTPUT':
        updateOutputHandler(m.body);
        break;
        
      default:
        print('Console error: unrecognized message type.');
    }
  }
  
  /// Updates the output field based on string messages passed in.
  void updateOutputHandler(String s) {
    output.appendText('up> $s');
    output.appendHtml('<br/>');

    // Autoscroll the new messages as they come in.
    output.scrollTop = output.scrollHeight;
  }

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == CONSOLE_COMMAND)
        .listen((um) => updateOutputHandler(um.body));
    
    cs.stream
        .where((m) => m.dest == 'CONSOLE')
        .listen((m) => processMessage(m));
    
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