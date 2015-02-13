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
  Terminal term;

  DivElement console;
  AnchorElement consoleButton;
  AnchorElement themeButton;
  
  bool lightTheme;
  bool consoleSelected;
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    lightTheme = false;
    consoleSelected = false;

    console = querySelector('#console');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
    term = new Terminal(console);
    
    registerConsoleEventHandlers();

    cs.add(new CommanderMessage('CLIENT', 'CONSOLE_READY'));
  }
  
  /// Toggles between a Solarized dark and light theme.
  void toggleTheme() {
    if (lightTheme) {
      console.style.backgroundColor = '#002b36'; // base-green
      lightTheme = false;
    } else {
      console.style.backgroundColor = '#fdf6e3'; // base-tan
      lightTheme = true;
    }
  }
  
  /// Process messages that Console has picked up according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CONNECTED':
        //updateOutputHandler('Connected to updroid!');
        break;
        
      case 'DISCONNECTED':
        //updateOutputHandler('Updroid disconnected.');
        break;

      case 'OUTPUT':
        //updateOutputHandler(m.body);
        break;
        
      default:
        print('Console error: unrecognized message type: ' + m.type);
    }
  }
  
  void handleInput(KeyboardEvent e) {
    int key = e.keyCode;

    // Carriage Return (13) => New Line (10).
    if (key == 13) {
      key = 10;
    }
    
    // Apply the Shift modifier if applicable.
    if (!e.shiftKey) {
      if (MODIFIABLE_KEYS.containsKey(key)) {
        key = MODIFIABLE_KEYS[key];
      }
    }
    
    // Don't let solo modifier keys through (Shift=16, Ctrl=17, Meta=91, Alt=18).
    if (key != 16 && key != 17 && key != 91 && key != 18) {
      //print(key);
      ws.send('[[CONSOLE_INPUT]]' + key.toString());
    }
  }

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'CONSOLE_OUTPUT')
        .listen((um) => term.stdout.add(um.body));

    cs.stream
        .where((m) => m.dest == 'CONSOLE' || m.dest == 'ALL')
        .listen((m) => processMessage(m));

    console.onKeyUp.listen((e) => handleInput(e));
    
    // TODO: figure out a way to deselect the console.
    console.onClick.listen((e) {
      consoleSelected = true;
      print(consoleSelected);
    });
    
    window.onClick.listen((e) {
      if (e.target.classes.contains('termrow') || e.target.id == 'console') {

      } else {
        consoleSelected = false;
        print('click' + consoleSelected.toString());
      }
    });
    
    window.onMouseWheel.listen((wheelEvent) {
      print('wheel' + consoleSelected.toString());
      if (consoleSelected) {
        // Scrolling should target only the console.
        wheelEvent.preventDefault();
        print('scroll!' + wheelEvent.deltaY.toString());
      }
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
  }
}