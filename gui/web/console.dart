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
  Utf8Decoder utf8Decoder;

  DivElement console;
  AnchorElement consoleButton;
  AnchorElement themeButton;
  
  bool lightTheme;
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    utf8Decoder = new Utf8Decoder();
    
    lightTheme = false;

    console = querySelector('#console');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
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

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'CONSOLE_OUTPUT')
        .listen((um) => print(utf8Decoder.convert(um.body)));
    
    cs.stream
        .where((m) => m.dest == 'CONSOLE' || m.dest == 'ALL')
        .listen((m) => processMessage(m));
    
    console.onKeyUp.listen((e) {
      print(e.keyCode.toString());
      ws.send('[[CONSOLE_INPUT]]' + e.keyCode);
      
      if (e.ctrlKey && e.keyCode == 67) {
        print('Ctrl-C');
      }
    });
    
    // for debug
    console.onClick.listen((e) => print('selected!'));
    
    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
  }
}