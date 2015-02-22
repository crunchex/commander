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
  bool inputDone;
  
  List<int> inputString;
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    lightTheme = false;
    consoleSelected = false;
    inputDone = false;
    inputString = [];

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
    
    // keyCode behaves very oddly.
    if (!e.shiftKey) {
      if (NOSHIFT_KEYS.containsKey(key)) {
        key = NOSHIFT_KEYS[key];
      }
    } else {
      if (SHIFT_KEYS.containsKey(key)) {
        key = SHIFT_KEYS[key];
      }
    }

    // Carriage Return (13) => New Line (10).
    if (key == 13) {
      key = 10;
      inputDone = true;
    }

    // Don't let solo modifier keys through (Shift=16, Ctrl=17, Meta=91, Alt=18).
    if (key != 16 && key != 17 && key != 91 && key != 18) {
      inputString.add(key);
    }
    
    if (inputDone) {
      ws.send('[[CONSOLE_INPUT]]' + JSON.encode(inputString));
      inputString = [];
      inputDone = false;
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
    
    console.onKeyDown.listen((e) {
      if (e.keyCode == 8) e.preventDefault();
    });
    
    // TODO: figure out a way to deselect the console.
    console.onClick.listen((e) {
      consoleSelected = true;
    });
    
    window.onClick.listen((e) {
      if (e.target.classes.contains('termrow') || e.target.id == 'console') {
        return;
      }

      consoleSelected = false;
    });
    
    window.onMouseWheel.listen((wheelEvent) {
      if (!consoleSelected) return;

      // Scrolling should target only the console.
      wheelEvent.preventDefault();
      
      if (wheelEvent.deltaY < 0) {
        if (!term.atTop) {
          term.bufferIndex--;
          term.refreshDisplay();
        }
      } else {
        if (!term.atBottom) {
          term.bufferIndex++;
          term.refreshDisplay();
        }
      }
    });
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
  }
}