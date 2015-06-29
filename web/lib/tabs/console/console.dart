library updroid_console;

import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'package:terminal/terminal.dart';
import 'package:terminal/theme.dart';

import '../../mailbox.dart';
import '../tab_controller.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket] into an UpDroid Commander tab.
class UpDroidConsole extends TabController {
  static const String className = 'UpDroidConsole';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Settings', 'items': [
        {'type': 'toggle', 'title': 'Invert'},
        {'type': 'toggle', 'title': 'Cursor Blink'}]}
    ];
    return menu;
  }

  WebSocket _ws;
  Terminal _term;

  AnchorElement _themeButton;
  AnchorElement _blinkButton;

  Timer _resizeTimer;

  UpDroidConsole(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Console', getMenuConfig(), cs, true) {

  }

  void setUpController() {
    _themeButton = view.refMap['invert'];
    _blinkButton = view.refMap['cursor-blink'];

    _term = new Terminal(view.content)
      ..scrollSpeed = 3
      ..cursorBlink = true
      ..theme = new Theme.SolarizedDark();
  }

  /// Toggles between a Solarized dark and light theme.
  void _toggleTheme() {
    _term.theme = _term.theme.name == 'solarized-light' ? new Theme.SolarizedDark() : new Theme.SolarizedLight();
  }

  /// Toggles cursor blink on/off.
  void _toggleBlink() {
    _term.cursorBlink = _term.cursorBlink ? false : true;
  }

  void _startPty(UpDroidMessage um) {
    List<int> size = _term.calculateSize();
    _term.resize(size[0], size[1]);
    mailbox.ws.send('[[START_PTY]]${size[0]}x${size[1] - 1}');
  }

  /// Starts a secondary WebSocket with direct access to the pty spawned by CmdrPty.
  void _initWebSocket(UpDroidMessage um) {
    String url = window.location.host;
    url = url.split(':')[0];
    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    _resetWebSocket('ws://' + url + ':12060/${className.toLowerCase()}/$id/cmdr-pty');
  }

  void _resetWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);
    _ws.binaryType = "arraybuffer";

    _ws.onMessage.listen((e) {
      ByteBuffer buf = e.data;
      _term.stdout.add(buf.asUint8List());
    });

    _ws.onError.listen((e) {
      print('Console-$id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _resetWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  void _resizeEvent(CommanderMessage m) {
    List newSize = m.body.split('x');
    int newRow = int.parse(newSize[0]);
    int newCol = int.parse(newSize[1]);
    _term.resize(newRow, newCol);
    // _cols must be $COLUMNS - 1 or we see some glitchy stuff. Also rows.
    mailbox.ws.send('[[RESIZE]]' + '${newRow}x${newCol - 1}');
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {
    mailbox.registerCommanderEvent('RESIZE', _resizeEvent);

    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'START_PTY', _startPty);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'PTY_READY', _initWebSocket);
  }

  /// Sets up the event handlers for the console.
  void registerEventHandlers() {
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

    window.onResize.listen((e) {
      if (view.content.parent.classes.contains('active')) {
        // Timer prevents a flood of resize events slowing down the system and allows the window to settle.
        if (_resizeTimer != null) _resizeTimer.cancel();
        _resizeTimer = new Timer(new Duration(milliseconds: 500), () {
          List<int> newSize = _term.calculateSize();
          cs.add(new CommanderMessage('UPDROIDCONSOLE', 'RESIZE', body: '${newSize[0]}x${newSize[1]}'));
        });
      }
    });
  }

  void cleanUp() {
    _ws.close();
  }
}

//void main() {
////  int id = message[0];
////  int column = message[1];
////  bool active = message[2];
//
//  StreamController<CommanderMessage> cs = new StreamController<CommanderMessage>.broadcast();
//
//  new UpDroidConsole(1, 2, cs, active: true);
//}