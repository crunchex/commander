library updroid_client;

import 'dart:html';
import 'dart:convert';

import 'panels/panel_controller.dart';
import 'panels/explorer/explorer.dart';
import 'mailbox.dart';
import 'column_controller.dart';

class UpDroidClient {
  String _config;
  List<PanelController> _panels;
  List<ColumnController> _columnControllers;

  bool disconnectAlert = false;

  Mailbox _mailbox;

  UpDroidClient() {
    _config = _getConfig();

    // TODO: figure out how to handle panels along with the logo.
    _panels = [];
    _columnControllers = [];

    _mailbox = new Mailbox('UpDroidClient', 1);

    _registerMailbox();
    _registerEventHandlers(_getConfig());
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'GET_CONFIG', _getClientConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _initializeClient);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers(String config) {

  }

  //\/\/ Mailbox Handlers /\/\//

  void _getClientConfig(UpDroidMessage um) => _mailbox.ws.send('[[CLIENT_CONFIG]]');

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  void _initializeClient(UpDroidMessage um) {
    List config = JSON.decode(_config);

    for (int i = 0; i < 1; i++) {
      for (Map panel in config[i]) {
        _openPanel(i, panel['id'], panel['class']);
      }
    }

    // Determine the column width for each tab column depending on how many
    // there are in the config, minus 1 for the explorer panel.
//    const bootStrapColumnsForTabs = 10;
//    const numberOfPanels = 1;
//    int columnWidth = bootStrapColumnsForTabs ~/ (config.length - numberOfPanels);

    for (int i = 1; i < config.length; i++) {
      ColumnController controller = new ColumnController(i, ColumnState.NORMAL, config[i], _mailbox, _getAvailableId);
      _columnControllers.add(controller);

      controller.columnEvents.listen((ColumnState newState) {
        if (newState == ColumnState.MAXIMIZED) {
          _columnControllers.where((c) => c != controller).forEach((c) => c.minimize(false));
        } else if (newState == ColumnState.MINIMIZED) {
          _columnControllers.where((c) => c != controller).forEach((c) => c.maximize(false));
        } else {
          _columnControllers.where((c) => c != controller).forEach((c) => c.resetToNormal(false));
        }
      });
    }
  }

  void _closeTabFromServer(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String tabType = idList[0];
    int tabId = int.parse(idList[1]);

    for (ColumnController controller in _columnControllers) {
      // Break once one of the controllers finds the tab to close.
      if (controller.findAndCloseTab(tabId, tabType)) break;
    }
  }

  //\/\/ Event Handlers /\/\/

  //\/\/ Misc Functions /\/\//

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
    _columnControllers.forEach((controller) {
      ids.addAll(controller.returnIds(className));
    });

    // Find the lowest unused ID possible.
    int id = 0;
    bool found = false;
    while (!found) {
      id++;
      if (!ids.contains(id)) break;
    }

    return id;
  }

  void _openPanel(int column, int id, String className) {
    if (_panels.length >= 1) return;

    if (_panels.isNotEmpty) {
      for (var panel in _panels) {
        panel.makeInactive();
      }
    }

    if (className == 'UpDroidExplorer') {
      _mailbox.ws.send('[[OPEN_PANEL]]' + '$column-$id-$className');
      _panels.add(new UpDroidExplorer(id, column));
    }
  }
}
