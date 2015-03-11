part of updroid_client;

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket], into an UpDroid Commander tab.
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

    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    String url = window.location.host;
    url = url.split(':')[0];
    _initWebSocket('ws://' + url + ':1206$consoleNum/pty');

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