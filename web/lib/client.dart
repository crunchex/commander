library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'tabs/tab_controller.dart';
import 'panels/explorer/explorer.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'mailbox.dart';

class UpDroidClient {
  List<List<dynamic>> _columns;
  String _config;

  AnchorElement _newButtonLeft;
  AnchorElement _newButtonRight;

  bool disconnectAlert = false;

  Mailbox _mailbox;

  UpDroidClient() {
    _config = _getConfig();

    _columns = [[], [], []];

    _newButtonLeft = querySelector('#column-1-new');
    _newButtonRight = querySelector('#column-2-new');

    _mailbox = new Mailbox('UpDroidClient', 1);

    _registerMailbox();
    _registerEventHandlers(_getConfig());

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

        if (tab.fullName == className) ids.add(tab.id);
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
  void _initializeClient(UpDroidMessage um) {
    List config = JSON.decode(_config);

    for (int i = 0; i < 1; i++) {
      for (Map panel in config[i]) {
        _openPanel(i, panel['id'], panel['class']);
      }
    }

    for (int i = 1; i < config.length; i++) {
      for (Map tab in config[i]) {
        _openTab(i, tab['id'], tab['class']);
      }
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
       _columns[column].add(new UpDroidExplorer(id, column));
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
      _columns[column].add(new UpDroidEditor(id, column));
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
      _columns[column].add(new UpDroidConsole(id, column));
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void _closeTabFromServer(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String type = idList[0];
    int num = int.parse(idList[1]);

    // Find the tab to remove and remove it.
    // Also take note of the column it was found in.
    int col;
    for (int currColumn = 0; currColumn < 3; currColumn++) {
      for (int currContainer = 0; currContainer < _columns[currColumn].length; currContainer++) {
        if (_columns[currColumn][currContainer].fullName == type && _columns[currColumn][currContainer].id == num) {
          _columns[currColumn].removeAt(currContainer);
          col = currColumn;
        }
      }
    }

    // Make all tabs in that column inactive except the last.
    _columns[col].forEach((TabController tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_columns[col].length > 0) _columns[col].last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + id);
  }

  void _openTabFromButton(int column, String className) {
    int id = _getAvailableId(className);
    _openTab(column, id, className);
  }

  void _getClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'GET_CONFIG', _getClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _initializeClient);
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
