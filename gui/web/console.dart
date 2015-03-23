library updroid_console;

import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'lib/updroid_message.dart';
import 'lib/terminal/terminal.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket], into an UpDroid Commander tab.
class UpDroidConsole {
  static const String className = 'UpDroidConsole';

  StreamController<CommanderMessage> _cs;
  int _consoleNum;
  WebSocket _ws;
  Terminal _term;

  DivElement _console;
  AnchorElement _consoleButton;
  AnchorElement _themeButton;
  AnchorElement _blinkButton;
  bool _blink;
  bool _lightTheme;

  UpDroidConsole(int consoleNum, StreamController<CommanderMessage> cs) {
    _consoleNum = consoleNum;
    _cs = cs;

    _console = querySelector('#console-$consoleNum');
    _consoleButton = querySelector('#button-console-$consoleNum');
    _themeButton = querySelector('#button-console-theme-$consoleNum');
    _blinkButton = querySelector('#button-console-blink-$consoleNum');

    _term = new Terminal(_console);
    _term
        ..scrollSpeed = 3
        ..cursorBlink = true
        ..theme = new Theme.SolarizedDark();

    _blink = true;
    _lightTheme = false;

    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    String url = window.location.host;
    url = url.split(':')[0];
    _initWebSocket('ws://' + url + ':1206$consoleNum/pty');

    _registerConsoleEventHandlers();
  }

  /// Toggles between a Solarized dark and light theme.
  void _toggleTheme() {
    _term.theme = _lightTheme ? new Theme.SolarizedDark() : new Theme.SolarizedLight();
    _lightTheme = !_lightTheme;
  }

  /// Toggles cursor blink on/off.
  void _toggleBlink() {
    _term.cursorBlink = _blink ? false : true;
    _blink = !_blink;
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

    _blinkButton.onClick.listen((e) {
      _toggleBlink();
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