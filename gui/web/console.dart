library updroid_console;

import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'package:collection/equality.dart';
import 'package:terminal/terminal.dart';
import 'package:terminal/theme.dart';

import 'lib/updroid_message.dart';
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
  WebSocket _wsMain;
  Terminal _term;

  DivElement _console;
  AnchorElement _closeTabButton;
  AnchorElement _themeButton;
  AnchorElement _blinkButton;

  UpDroidConsole(this.num, int col, StreamController<CommanderMessage> cs, {bool active: false}) {
    _col = col;
    _cs = cs;

    setUpTabHandle(num, _col, 'Console', active);
    setUpTabContainer(num, _col, 'Console', _getMenuConfig(), active).then((Map configRefs) {
      _console = configRefs['content'];
      _console.tabIndex = 0;
      _closeTabButton = configRefs['close-tab'];
      _themeButton = configRefs['invert'];
      _blinkButton = configRefs['cursor-blink'];

      _term = new Terminal(_console)
        ..scrollSpeed = 3
        ..cursorBlink = true
        ..theme = new Theme.SolarizedDark();

      String url = window.location.host;
      url = url.split(':')[0];
      // Create the server <-> client [WebSocket].
      // Port 12060 is the default port that UpDroid uses.
      _wsMain = new WebSocket('ws://' + url + ':12060/pty/$num');
      // window.location.host returns whatever is in the URL bar (including port).
      // Since the port here needs to be dynamic, the default needs to be replaced.
      _initWebSocket('ws://' + url + ':1206$num/pty');

      _registerConsoleEventHandlers();
    });
  }

  /// Process messages according to the type.
  void _processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'RESIZE':
        List newSize = m.body.split('x');
        int newRow = int.parse(newSize[0]);
        int newCol = int.parse(newSize[1]);
        _term.resize(newRow, newCol);
        // _cols must be $COLUMNS - 1 or we see some glitchy stuff. Also rows.
        _wsMain.send('[[RESIZE]]' + '${newRow - 1}x${newCol - 1}');
        break;

      default:
        print('Console error: unrecognized message type: ' + m.type);
    }
  }

  /// Toggles between a Solarized dark and light theme.
  void _toggleTheme() {
    _term.theme = _term.theme.name == 'solarized-light' ? new Theme.SolarizedDark() : new Theme.SolarizedLight();
  }

  /// Toggles cursor blink on/off.
  void _toggleBlink() {
    _term.cursorBlink = _term.cursorBlink ? false : true;
  }

  /// Sets up the event handlers for the console.
  void _registerConsoleEventHandlers() {
    _cs.stream.where((m) => m.dest == 'CONSOLE').listen((m) => _processMessage(m));

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

    tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_col}_UpDroidConsole'));
    });

    _console.onClick.listen((e) {
//      List<int> oldSize = _term.currentSize();
//      List<int> newSize = _term.calculateSize();
//
//      if (const ListEquality().equals(oldSize, newSize)) return;
//
//      _cs.add(new CommanderMessage('CONSOLE', 'RESIZE', body: '${newSize[0]}x${newSize[1]}'));
    });

    window.onResize.listen((e) {
      if (_console.parent.classes.contains('active')) {
        List<int> newSize = _term.calculateSize();
        _cs.add(new CommanderMessage('CONSOLE', 'RESIZE', body: '${newSize[0]}x${newSize[1]}'));
      }
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
        {'type': 'toggle', 'title': 'Invert'},
        {'type': 'toggle', 'title': 'Cursor Blink'}]}
    ];
    return menu;
  }
}