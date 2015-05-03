library updroid_console;

import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'package:terminal/terminal.dart';
import 'package:terminal/theme.dart';

import '../updroid_message.dart';
import '../tab.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket] into an UpDroid Commander tab.
class UpDroidConsole {
  static const String className = 'UpDroidConsole';
  String type = 'UpDroidConsole';

  int num;
  int _col;
  StreamController<CommanderMessage> _cs;

  TabView _view;
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

    TabView.createTabView(num, _col, 'Console', active, _getMenuConfig()).then((tabView) {
      _view = tabView;
      setUpController();
    });
  }

  void setUpController() {
    _console = _view.content;
    _console.tabIndex = 0;
    _console.contentEditable = "true";

    _closeTabButton = _view.refMap['close-tab'];
    _themeButton = _view.refMap['invert'];
    _blinkButton = _view.refMap['cursor-blink'];

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

    _wsMain.onOpen.listen((e) {
      List<int> size = _term.currentSize();
      _wsMain.send('[[RESIZE]]' + '${size[0] - 1}x${size[1] - 1}');
    });

    _ws.onMessage.listen((e) {
      ByteBuffer buf = e.data;
      _term.stdout.add(buf.asUint8List());
    });

    _term.stdin.stream.listen((data) {
      _ws.sendByteBuffer(new Uint8List.fromList(data).buffer);
    });

    _closeTabButton.onClick.listen((e) {
      _view.destroy();
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

    _view.tabHandleButton.onDoubleClick.listen((e) {
      e.preventDefault();
      _cs.add(new CommanderMessage('CLIENT', 'OPEN_TAB', body: '${_col}_UpDroidConsole'));
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

  /// TODO: put these into super class.
  void makeActive() => _view.makeActive();
  void makeInactive() => _view.makeInactive();

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