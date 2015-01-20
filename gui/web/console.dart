part of updroid_client;

const String CONSOLE_HELP_TEXT =
r'''
Console only accepts commands that are non-interactive and command-line based.
For example:
  pwd, ls -ls

Programs that are curses-based, such as htop, will not work.
''';

/// [UpDroidConsole] functions like a trimmed down terminal that allows the
/// user to pass in white-listed commands to the server and view their output.
/// It is not meant to be a complete terminal emulator like xterm.
class UpDroidConsole {
  WebSocket ws;
  StreamController<CommanderMessage> cs;

  DivElement console;
  SpanElement prompt;
  InputElement input;
  AnchorElement consoleButton;
  AnchorElement themeButton;
  
  bool lightTheme;
  bool processRunning;
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    lightTheme = false;
    processRunning = false;

    console = querySelector('#console');
    prompt = querySelector('#prompt');
    input = querySelector('#input');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
    registerConsoleEventHandlers();
    
    // This is a hack to account for the fact that Console is set up
    // after the initial connection is made and thus, misses the first
    // message.
    updateOutputHandler('Connected to updroid.');
    cs.add(new CommanderMessage('CLIENT', 'CONSOLE_READY'));
  }
  
  /// Toggles between a Solarized dark and light theme.
  void toggleTheme() {
    if (lightTheme) {
      console.style.backgroundColor = '#002b36'; // base-green
      querySelectorAll('.pre-output').style.color = '#93a1a1'; // light-grey
      querySelectorAll('.prompt').style.color = '#859900'; // green
      querySelectorAll('.user-command').style.color = '#268bd2';  // blue
      lightTheme = false;
    } else {
      console.style.backgroundColor = '#fdf6e3'; // base-tan
      querySelectorAll('.pre-output').style.color = '#586e75'; // dark-grey
      querySelectorAll('.prompt').style.color = '#b58900'; // yellow
      querySelectorAll('.user-command').style.color = '#dc322f';  // red
      lightTheme = true;
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
    // Generate the new line.
    PreElement newLine = new PreElement();
    newLine
        ..text = s
        ..classes.add('pre-output')
        ..style.color = lightTheme ? '#586e75' : '#93a1a1';
    
    console.children.insert(console.children.length - 1, newLine);

    // Autoscroll the new messages as they come in.
    console.scrollTop = console.scrollHeight;
  }
  
  /// Copies the prompt and user's command and inserts them above the current output.
  /// This is simply to mimic how a real terminal works.
  Future copyCommand(String cmd) {
    Completer completer = new Completer();
    
    SpanElement userInput = new SpanElement();

    SpanElement prompt = new SpanElement();
    prompt
        ..text = '[up, droid!] '
        ..classes.add('prompt')
        ..style.color = lightTheme ? '#b58900' : '#859900';
    
    SpanElement command = new SpanElement();
    command
        ..text = cmd
        ..classes.add('user-command')
        ..style.color = lightTheme ? '#dc322f' : '#268bd2';
    
    userInput.children.add(prompt);
    userInput.children.add(command);

    console.children.insert(console.children.length - 1, userInput);
    console.children.insert(console.children.length - 1, new BRElement());
    
    // Autoscroll the new messages as they come in.
    console.scrollTop = console.scrollHeight;
    
    completer.complete(cmd);
    return completer.future;
  }
  
  /// Handles when a user enters new input or runs a new command.
  void processInput() {
    if (processRunning) {
      ws.send('[[CONSOLE_INPUT]]' + input.value.trim() + '\n');
      input.value = "";
      return;
    }
    
    copyCommand(input.value.trim()).then((cmd) {
      prompt.classes.add('prompt-hidden');
      prompt.classes.remove('prompt');
      input.value = "";
      ws.send('[[CONSOLE_COMMAND]]' + cmd);
      processRunning = true;
    });
  }

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'CONSOLE_OUTPUT')
        .listen((um) => updateOutputHandler(um.body));
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'CONSOLE_EXIT')
        .listen((um) {
            //updateOutputHandler('Process exited with code: ' + um.body);
            prompt.classes.remove('prompt-hidden');
            prompt.classes.add('prompt');
            processRunning = false;
        });
    
    cs.stream
        .where((m) => m.dest == 'CONSOLE')
        .listen((m) => processMessage(m));
    
    input.onKeyUp.listen((e) {
      var keyEvent = new KeyEvent.wrap(e);
      if (keyEvent.keyCode == KeyCode.ENTER) {
        RegExp allWhitespace = new RegExp(r'^[\s]*$');
        if (!input.value.contains(allWhitespace)) {
          processInput();
        }
      }
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
    
    consoleButton.onClick.listen((e) => input.focus());
    console.onClick.listen((e) => input.focus());
  }
}