library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'panels/explorer/explorer.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'column_view.dart';
import 'modal/modal.dart';
import 'mailbox.dart';

class UpDroidClient {
  StreamController<CommanderMessage> _cs;

  String _config;
  List<List<dynamic>> _columns;
  Mailbox _mailbox;

  bool disconnectAlert = false;

  UpDroidClient() {
    _config = _getConfig();
    _columns = [];

    // Create the intra-client message stream.
    // The classes use this to communicate with each other.
    _cs = new StreamController<CommanderMessage>.broadcast();
    _mailbox = new Mailbox('UpDroidClient', 1, _cs);

    _registerMailbox();

    _pulseFeedback(querySelector('#feedback-button'));
  }

  void _pulseFeedback(AnchorElement feedbackButton) {
    // Initial pulse - 30 seconds in.
    new Timer(new Duration(seconds: 30), () {
      feedbackButton.classes.add('feedback-bold');
      new Timer(new Duration(milliseconds: 500), () {
        feedbackButton.classes.remove('feedback-bold');
      });
    });

    // Every 5 minutes after.
    new Timer.periodic(new Duration(minutes: 5), (timer) {
      feedbackButton.classes.add('feedback-bold');
      new Timer(new Duration(milliseconds: 500), () {
        feedbackButton.classes.remove('feedback-bold');
      });
    });
  }

  /// Returns a [Map] of all the tabs that [UpDroidClient] needs to spawn,
  /// where key = side|left|right| and value = a list of IDs and UpDroidTab.className.
  /// TODO: this function should retrieve information from some defaults or
  /// a user account settings save.
  String _getConfig([String strConfig = '']) {
    // TODO: the default should eventually be JSON'd.
    //if (strConfig == null) strConfig = UpDroidClient.defaultConfig;
    if (strConfig != '') return strConfig;

    List listConfig = [
      [{'id': 1, 'class': 'UpDroidExplorer'}],
      [{'id': 1, 'class': 'UpDroidEditor'}],
      [{'id': 1, 'class': 'UpDroidConsole'}]
    ];

    return JSON.encode(listConfig);
  }

  int _getAvailableId(String className) {
    List ids = [];

    // Add all used ids for [className] to ids.
    for (int i = 1; i <= 2; i++) {
      _columns[i].forEach((tab) {

        if (tab.tabType == className) ids.add(tab.id);
      });
    }

    // Find the lowest unused ID possible.
    int id = 0;
    bool found = false;
    while (!found) {
      id++;
      if (!ids.contains(id)) break;
    }

    return id;
  }

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  Future _initializeColumns(UpDroidMessage um) async {
    List config = JSON.decode(_config);

    for (int i = 0, i < 3; i++) {
      int width = (i == 0) ? 2 : 5;
      ColumnView.createColumnView(i, width).then((columnView) {
        if (width == 2) {}
      })
    }

    for (Map panel in config[i]) {
      _openPanel(i, panel['id'], panel['class']);
    }

    for (Map tab in config[i]) {
      _openTab(i, tab['id'], tab['class']);
    }

    _registerEventHandlers(_getConfig());
  }

  void _alertDisconnect(CommanderMessage m) {
    if (disconnectAlert == false) {
      window.alert("UpDroid Commander has lost connection to the server.");
      disconnectAlert = true;
    }
  }

  void _openPanel(int column, int id, String className) {
    if (_columns[column].length >= 1) return;

    if (_columns[column].isNotEmpty) {
      for (var panel in _columns[column]) {
        panel.makeInactive();
      }
    }

    if (className == 'UpDroidExplorer') {
      _mailbox.ws.send('[[OPEN_PANEL]]' + '$column-$id-$className');
       _columns[column].add(new UpDroidExplorer(id, column, _cs));
    }
  }

  void _openTab(int column, int id, String className) {
    if (_columns[column].length >= 4) return;

    if (_columns[column].isNotEmpty) {
      for (var tab in _columns[column]) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidEditor(id, column, _cs));
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidCamera(id, column));
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className');
      _columns[column].add(new UpDroidTeleop(id, column));
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$column-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _columns[column].add(new UpDroidConsole(id, column, _cs));
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void _closeTab(CommanderMessage m) {
    String id = m.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Find the tab to remove and remove it.
    // Also take note of the column it was found in.
    int col;
    for (int i = 1; i <= 2; i++) {
      for (int j = 0; j < _columns[i].length; j++) {
        if (_columns[i][j].tabType == type && _columns[i][j].id == num) {
          _columns[i].removeAt(j);
          col = i;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    for (int j = 1; j < _columns[col].length; j++) {
      _columns[col][j].makeInactive();
    }

    if (_columns[col].length > 0) _columns[col].last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + id);
  }

  void _closeTabFromServer(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Find the tab to remove and remove it.
    // Also take note of the column it was found in.
    int col;
    for (int i = 0; i <= 1; i++) {
      for (int j = 0; j < _columns[i].length; j++) {
        if (_columns[i][j].tabType == type && _columns[i][j].id == num) {
          _columns[i].removeAt(j);
          col = i;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    for (int j = 0; j < _columns[col].length; j++) {
      _columns[col][j].makeInactive();
    }

    if (_columns[col].length > 0) _columns[col].last.makeActive();
  }

  void _openTabFromButton(int column, String className) {
    int id = _getAvailableId(className);
    _openTab(column, id, className);
  }

  void _getClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');

  void _registerMailbox() {
    _mailbox.registerCommanderEvent('CLOSE_TAB', _closeTab);
    _mailbox.registerCommanderEvent('SERVER_DISCONNECT', _alertDisconnect);

    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'GET_CONFIG', _getClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _initializeColumns);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers(String config) {
    _newButtonLeft.onClick.listen((e) {
      e.preventDefault();
      if (_columns[1].length >= 4) return;

      new UpDroidOpenTabModal(1, _openTabFromButton);
    });

    _newButtonRight.onClick.listen((e) {
      e.preventDefault();
      if (_columns[2].length >= 4) return;

      new UpDroidOpenTabModal(2, _openTabFromButton);
    });
  }
}
