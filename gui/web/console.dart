library updroid_console;

import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'lib/updroid_message.dart';
import 'lib/terminal/terminal.dart';
import 'tab.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket], into an UpDroid Commander tab.
class UpDroidConsole extends UpDroidTab {
  static const String className = 'UpDroidConsole';
  String type = 'UpDroidConsole';

  StreamController<CommanderMessage> _cs;
  int num;
  int _col;
  WebSocket _ws;
  Terminal _term;

  DivElement _console;
  AnchorElement _closeTabButton;
  AnchorElement _themeButton;
  AnchorElement _blinkButton;
  bool _blink;
  bool _lightTheme;

  UpDroidConsole(this.num, int col, StreamController<CommanderMessage> cs, {bool active: false}) {
    _col = col;
    _cs = cs;

    setUpTabHandle(num, _col, 'Console', active);
    setUpTabContainer(num, _col, 'Console', _getMenuConfig(), active).then((Map configRefs) {
      _console = configRefs['content'];
      _closeTabButton = configRefs['close-tab'];
      _themeButton = configRefs['theme'];
      _blinkButton = configRefs['cursor-blink'];

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
      _initWebSocket('ws://' + url + ':1206$num/pty');

      _registerConsoleEventHandlers();
    });
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

    _closeTabButton.onClick.listen((e) {
      destroyTab();
      _cs.add(new CommanderMessage('CLIENT', 'CLOSE_TAB', body: '${type}_$num'));
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
      print('Console-$num disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });

    _ws.onError.listen((e) {
      print('Console-$num disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  List _getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Theme'},
        {'type': 'toggle', 'title': 'Cursor Blink'}]}
    ];
    return menu;
  }
}