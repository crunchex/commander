library updroid_client;

import 'dart:async';
import 'dart:html';
import 'dart:convert';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:quiver/async.dart';

import 'panels/panel_controller.dart';
import 'panels/explorer/explorer.dart';
import 'column_controller.dart';
import 'tab_interface.dart';

class UpDroidClient {
  String _config;
  List<PanelController> _panels;
  List<ColumnController> _columnControllers;
  Map _tabsInfo;
  Completer _gotConfig, _gotTabInfo;

  bool disconnectAlert = false;

  Mailbox _mailbox;

  UpDroidClient() {
    _gotConfig = new Completer();
    _gotTabInfo = new Completer();
    FutureGroup readyForInitialization = new FutureGroup();
    readyForInitialization.add(_gotConfig.future);
    readyForInitialization.add(_gotTabInfo.future);

    _config = _getConfig();
    _gotConfig.complete();

    // TODO: figure out how to handle panels along with the logo.
    _panels = [];
    _columnControllers = [];

    _mailbox = new Mailbox('UpDroidClient', 1);

    _registerMailbox();
    _registerEventHandlers(_getConfig());

    readyForInitialization.future.then((_) => _initializeClient());
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'MAKE_REQUESTS', _makeInitialRequests);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'TABS_INFO', _refreshTabsInfo);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _setUpConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLONE_TAB', _cloneTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'MOVE_TAB', _moveTabFromServer);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers(String config) {

  }

  //\/\/ Mailbox Handlers /\/\//

  void _makeInitialRequests(UpDroidMessage um) {
    _mailbox.ws.send('[[REQUEST_TABSINFO]]');
    _mailbox.ws.send('[[CLIENT_CONFIG]]');
  }

  void _setUpConfig(UpDroidMessage um) {
//    Completer c = new Completer();
//    _gotConfig = c.future;
//    c.complete();
  }

  void _refreshTabsInfo(UpDroidMessage um) {
    _tabsInfo = JSON.decode(um.body);
    _gotTabInfo.complete();
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

  void _cloneTabFromServer(UpDroidMessage um) {
    String id = um.body;
    List idList = id.split('_');
    String tabType = idList[0];
    int col = int.parse(idList[2]);

    Map<String, String> tabInfo;
    _tabsInfo.keys.forEach((String key) {
      tabInfo = _tabsInfo[key];
      if (tabInfo.containsValue(tabType)) return;
    });

    _columnControllers[col == 1 ? 0 : 1].openTabFromModal(tabInfo);
  }

  void _moveTabFromServer(UpDroidMessage um) {
    List idList = um.body.split('_');
    String tabType = idList[0];
    int id = int.parse(idList[1]);

    // Working with indexes here, not the columnId.
    int oldColIndex = int.parse(idList[2]) - 1;
    int newColIndex = int.parse(idList[3]) - 1;

    // Don't go any further if a move request can't be done.
    if (!_columnControllers[newColIndex].canAddMoreTabs) {
      window.alert('Can\'t move tab. Already at max on this side.');
      return;
    }

    TabInterface tab = _columnControllers[oldColIndex].removeTab(tabType, id);
    _columnControllers[newColIndex].addTab(tab);
  }

  //\/\/ Event Handlers /\/\/

  //\/\/ Misc Functions /\/\//

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  void _initializeClient() {
    List config = JSON.decode(_config);

    for (int i = 0; i < 1; i++) {
      for (Map panel in config[i]) {
        _openPanel(i, panel['id'], panel['class']);
      }
    }

    // TODO: make the initial min-width more responsive to how the tabs start out initially.
    // For now we assume they start off 50/50.
    querySelector('body').style.minWidth = '1211px';

    for (int i = 1; i < config.length; i++) {
      ColumnController controller = new ColumnController(i, ColumnState.NORMAL, config[i], _mailbox, _tabsInfo, _getAvailableId);
      _columnControllers.add(controller);

      controller.columnStateChanges.listen((ColumnState newState) {
        if (newState == ColumnState.MAXIMIZED) {
          querySelector('body').style.minWidth = '770px';
          _columnControllers.where((c) => c != controller).forEach((c) => c.minimize(false));
        } else if (newState == ColumnState.MINIMIZED) {
          querySelector('body').style.minWidth = '770px';
          _columnControllers.where((c) => c != controller).forEach((c) => c.maximize(false));
        } else {
          querySelector('body').style.minWidth = '1211px';
          _columnControllers.where((c) => c != controller).forEach((c) => c.resetToNormal(false));
        }
      });

      controller.columnEvents.listen((ColumnEvent event) {
        if (event == ColumnEvent.LOST_FOCUS) {
          _columnControllers.firstWhere((c) => c != controller).getsFocus();
        }
      });
    }
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
      [{'id': 1, 'class': 'upcom-editor'}],
      [{'id': 1, 'class': 'upcom-console'}]
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
