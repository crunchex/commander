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

  UpDroidConsole(int consoleNum, StreamController<CommanderMessage> cs) {
    this.cs = cs;

    console = querySelector('#console-$consoleNum');
    consoleButton = querySelector('#button-console-$consoleNum');
    themeButton = querySelector('.button-console-theme');

    term = new Terminal(console);
    term
        ..scrollSpeed = 3
        ..theme = 'solarized-dark';

    lightTheme = false;

    initWebSocket('ws://localhost:1206$consoleNum/pty');
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
    ws.onMessage.listen((e) {
      ByteBuffer buf = e.data;
      term.stdout.add(buf.asUint8List());
    });

    term.stdin.stream.listen((data) {
      ws.sendByteBuffer(new Uint8List.fromList(data).buffer);
    });

    themeButton.onClick.listen((e) {
      toggleTheme();
      e.preventDefault();
    });
  }

  void initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";

    ws.onOpen.listen((e) => print('Console connected.'));

    ws.onClose.listen((e) {
      print('Console disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });

    ws.onError.listen((e) {
      print('Console disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }
}