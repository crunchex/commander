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
  static const String upcomName = 'upcom';
  static const String explorerRefName = 'upcom-explorer';

  List _config;
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

    // TODO: figure out how to handle panels along with the logo.
    _panels = [];
    _columnControllers = [];

    _mailbox = new Mailbox(upcomName, 1);

    _registerMailbox();
    _registerEventHandlers();

    readyForInitialization.future.then((_) => _initializeClient());
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'MAKE_REQUESTS', _makeInitialRequests);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'TABS_INFO', _refreshTabsInfo);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _setUpConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'REQUEST_TAB', _requestTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLONE_TAB', _cloneTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'MOVE_TAB', _moveTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'ISSUE_ALERT', _issueAlert);
    _mailbox.registerWebSocketEvent(EventType.ON_CLOSE, 'CLEAN_UP', _cleanUp);
  }

  /// Sets up external event handlers for the various Commander classes. These
  /// are mostly listening events for [WebSocket] messages.
  void _registerEventHandlers() {

  }

  //\/\/ Mailbox Handlers /\/\//

  void _makeInitialRequests(Msg um) {
    _mailbox.ws.send('[[REQUEST_TABSINFO]]');
    _mailbox.ws.send('[[CLIENT_CONFIG]]');
  }

  void _setUpConfig(Msg um) {
    _config = JSON.decode(um.body);
    _gotConfig.complete();
  }

  void _refreshTabsInfo(Msg um) {
    _tabsInfo = JSON.decode(um.body);
    _gotTabInfo.complete();
  }

  void _requestTabFromServer(Msg um) {
    int tabId = _getAvailableId(um.body);

    for (ColumnController controller in _columnControllers) {
      if (controller.canAddMoreTabs) {
        controller.openTab(tabId, _tabsInfo[um.body], true);
        break;
      }
    }
  }

  void _closeTabFromServer(Msg um) {
    String id = um.body;
    List idList = id.split(':');
    String refName = idList[0];
    int tabId = int.parse(idList[1]);

    for (ColumnController controller in _columnControllers) {
      // Break once one of the controllers finds the tab to close.
      if (controller.findAndCloseTab(tabId, refName)) break;
    }
  }

  void _cloneTabFromServer(Msg um) {
    String id = um.body;
    List idList = id.split(':');
    String refName = idList[0];
    int col = int.parse(idList[2]);

    _tabsInfo.keys.forEach((String key) {
      if (_tabsInfo[key].containsValue(refName)) {
        _columnControllers[col == 1 ? 0 : 1].openTabFromModal(_tabsInfo[key]);
      }
    });
  }

  void _moveTabFromServer(Msg um) {
    List idList = um.body.split(':');
    String refName = idList[0];
    int id = int.parse(idList[1]);

    // Working with indexes here, not the columnId.
    int oldColIndex = int.parse(idList[2]) - 1;
    int newColIndex = int.parse(idList[3]) - 1;

    // Don't go any further if a move request can't be done.
    if (!_columnControllers[newColIndex].canAddMoreTabs) {
      window.alert('Can\'t move tab. Already at max on this side.');
      return;
    }

    TabInterface tab = _columnControllers[oldColIndex].removeTab(refName, id);
    _columnControllers[newColIndex].addTab(tab);
  }

  void _issueAlert(Msg m) => window.alert(m.body);

  void _cleanUp(Msg m) {
    _panels.forEach((panel) {
      panel.cleanUp();
      // TODO: need a less hacky way to clean up the explorer stuff.
      // Right now we have to manually clean up *around* the logo button as it's
      // part of col-0.
      panel.view.tabHandle.remove();
      panel.view.tabContainer.remove();
    });

    _columnControllers.forEach((controller) => controller.cleanUp());

    String alertMessage = 'UpDroid Commander has lost connection to the server.';
    alertMessage = alertMessage + ' Please restart the server (if necessary) and refresh the page.';
    window.alert(alertMessage);
  }

  //\/\/ Event Handlers /\/\/

  //\/\/ Misc Functions /\/\//

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  void _initializeClient() {
    _openPanel(0, 1, explorerRefName);

    String userAgent = window.navigator.userAgent;
    if (userAgent.contains('Mobile')) {
      querySelectorAll('html,body,#column-0,#col-0-tab-content,.footer,.text-muted')
        .forEach((e) => e.classes.add('mobile'));
      window.scrollTo(0, 1);
      return;
    }

    // TODO: make the initial min-width more responsive to how the tabs start out initially.
    // For now we assume they start off 50/50.
    querySelector('body').style.minWidth = '1211px';

    for (int i = 0; i < _config.length; i++) {
      ColumnController controller = new ColumnController(i+1, ColumnState.NORMAL, _config[i], _mailbox, _tabsInfo, _getAvailableId);
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

  void _openPanel(int column, int id, String refName) {
    if (_panels.length >= 1) return;

    if (_panels.isNotEmpty) {
      for (var panel in _panels) {
        panel.makeInactive();
      }
    }

    if (refName == explorerRefName) {
      _mailbox.ws.send('[[OPEN_PANEL]]' + '$column:$id:$refName');
      _panels.add(new UpDroidExplorer(id, column));
    }
  }
}
