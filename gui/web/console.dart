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
  StreamController<CommanderMessage> _cs;
  int _consoleNum;
  WebSocket _ws;
  Terminal _term;

  DivElement _console;
  AnchorElement _consoleButton;
  AnchorElement _themeButton;
  bool _lightTheme;

  UpDroidConsole(int consoleNum, StreamController<CommanderMessage> cs) {
    _consoleNum = consoleNum;
    _cs = cs;

    _console = querySelector('#console-$consoleNum');
    _consoleButton = querySelector('#button-console-$consoleNum');
    _themeButton = querySelector('#button-console-theme-$consoleNum');

    _term = new Terminal(_console);
    _term
        ..scrollSpeed = 3
        ..theme = 'solarized-dark';

    _lightTheme = false;

    _initWebSocket('ws://localhost:1206$consoleNum/pty');
    _registerConsoleEventHandlers();

    cs.add(new CommanderMessage('CLIENT', 'CONSOLE_READY'));
  }

  /// Toggles between a Solarized dark and light theme.
  void _toggleTheme() {
    if (_lightTheme) {
      _term.theme = 'solarized-dark';
      _lightTheme = false;
    } else {
      _term.theme = 'solarized-light';
      _lightTheme = true;
    }
  }

  /// Sets up the event handlers for the console.
  void _registerConsoleEventHandlers() {
    _ws.onMessage.listen((e) {
      ByteBuffer buf = e.data;
      _term.stdout.add(buf.asUint8List());
    });

    _term.stdin.stream.listen((data) {
      _ws.sendByteBuffer(new Uint8List.fromList(data).buffer);
    });

    _themeButton.onClick.listen((e) {
      _toggleTheme();
      e.preventDefault();
    });
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);
    _ws.binaryType = "arraybuffer";

    _ws.onOpen.listen((e) => print('Console-$_consoleNum connected.'));

    _ws.onClose.listen((e) {
      print('Console-$_consoleNum disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });

    _ws.onError.listen((e) {
      print('Console-$_consoleNum disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }
}