import 'dart:html';
import 'dart:async';
import 'dart:typed_data';

import 'package:terminal/terminal.dart';
import 'package:terminal/theme.dart';

import '../mailbox.dart';
import '../updroid_message.dart';
import '../tab_view.dart';
import '../tab_controller.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket] into an UpDroid Commander tab.
class UpDroidConsole extends TabController {
  static const String className = 'UpDroidConsole';
  // Only use where space is constrained, otherwise use className.
  static const String shortName = 'Console';

  WebSocket _ws;
  Terminal _term;

  AnchorElement _closeTabButton;
  AnchorElement _themeButton;
  AnchorElement _blinkButton;

  Timer _resizeTimer;

  UpDroidConsole(int id, int col, StreamController<CommanderMessage> cs, {bool active: false}) : super(id, col, className, cs, active: active) {
    TabView.createTabView(id, col, className, shortName, active, _getMenuConfig()).then((tabView) {
      view = tabView;
      setUpController();
    });
  }

  void setUpController() {
    _closeTabButton = view.refMap['close-tab'];
    _themeButton = view.refMap['invert'];
    _blinkButton = view.refMap['cursor-blink'];

    _registerMailbox();

    _term = new Terminal(view.content)
      ..scrollSpeed = 3
      ..cursorBlink = true
      ..theme = new Theme.SolarizedDark();

    String url = window.location.host;
    url = url.split(':')[0];
    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    _initWebSocket('ws://' + url + ':1206$id/pty');

    _registerConsoleEventHandlers();
  }

  /// Toggles between a Solarized dark and light theme.
  void _toggleTheme() {
    _term.theme = _term.theme.name == 'solarized-light' ? new Theme.SolarizedDark() : new Theme.SolarizedLight();
  }

  /// Toggles cursor blink on/off.
  void _toggleBlink() {
    _term.cursorBlink = _term.cursorBlink ? false : true;
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);
    _ws.binaryType = "arraybuffer";

    _ws.onError.listen((e) {
      print('Console-$id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  void _initialResize(UpDroidMessage um) {
    List<int> size = _term.currentSize();
    mailbox.ws.send('[[RESIZE]]' + '${size[0]}x${size[1] - 1}');
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

  void _registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'FIRST_RESIZE', _initialResize);
    mailbox.registerCommanderEvent('RESIZE', _resizeEvent);
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

    view.cloneControl.onClick.listen((e) {
      e.preventDefault();
      cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${col}_${className}'));
    });

    // TODO: this should be in tab_controller somehow.
    view.closeControl.onClick.listen((e) {
      view.destroy();

      // This is specific to Console class.
      _ws.close();

      cs.add(new CommanderMessage('UPDROIDCLIENT', 'CLOSE_TAB', body: '${className}_$id'));
    });

    _closeTabButton.onClick.listen((e) {
      view.destroy();
      _ws.close();
      cs.add(new CommanderMessage('UPDROIDCLIENT', 'CLOSE_TAB', body: '${className}_$id'));
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

  void _consoleSpecificClose() {
    _ws.close();
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

//void main() {
////  int id = message[0];
////  int column = message[1];
////  bool active = message[2];
//
//  StreamController<CommanderMessage> cs = new StreamController<CommanderMessage>.broadcast();
//
//  new UpDroidConsole(1, 2, cs, active: true);
//}