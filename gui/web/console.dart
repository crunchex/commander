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
  
  UpDroidConsole(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;

    console = querySelector('#console');
    consoleButton = querySelector('#button-console');
    themeButton = querySelector('.button-console-theme');
    
    term = new Terminal(console);
    term
        ..scrollSpeed = 3
        ..theme = 'solarized-dark';
    
    lightTheme = false;
    
    registerConsoleEventHandlers();

    cs.add(new CommanderMessage('CLIENT', 'CONSOLE_READY'));
  }
  
  /// Toggles between a Solarized dark and light theme.
  void toggleTheme() {
    if (lightTheme) {
      term.theme = 'solarized-dark';
      lightTheme = false;
    } else {
      term.theme = 'solarized-light';
      lightTheme = true;
    }
  }

  /// Sets up the event handlers for the console.
  void registerConsoleEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'CONSOLE_OUTPUT')
        .listen((um) => term.stdout.add(um.body));
    
    term.stdin.stream.listen((data) => ws.send('[[CONSOLE_INPUT]]' + data));

    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
  }
}