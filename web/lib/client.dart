library updroid_client;

import 'dart:async';
import 'dart:html';
import 'dart:convert';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';
import 'package:quiver/async.dart';

import 'column_controller.dart';
import 'panel_column_controller.dart';
import 'tab_column_controller.dart';
import 'tab_column_view.dart';
import 'panel_column_view.dart';
import 'plugin_interface.dart';

class UpDroidClient {
  static const String upcomName = 'upcom';
  static const String explorerRefName = 'upcom-explorer';

  List _config;
  List<PanelColumnController> _panelColumnControllers;
  List<TabColumnController> _tabColumnControllers;
  Map _panelsInfo, _tabsInfo;
  Map<String, List<int>> _tabIds;
  Completer _gotConfig, _gotPluginsInfo;

  bool disconnectAlert = false;

  Mailbox _mailbox;

  UpDroidClient() {
    _gotConfig = new Completer();
    _gotPluginsInfo = new Completer();
    FutureGroup readyForInitialization = new FutureGroup();
    readyForInitialization.add(_gotConfig.future);
    readyForInitialization.add(_gotPluginsInfo.future);

    // TODO: figure out how to handle panels along with the logo.
    _panelColumnControllers = [];
    _tabColumnControllers = [];

    // Since tabs can be created asyncronously via tab columns, IDs need to be
    // registered to a central pool as soon as they are created.
    _tabIds= {};

    _mailbox = new Mailbox(upcomName, 1);

    _registerMailbox();
    _registerEventHandlers();

    readyForInitialization.future.then((_) => _initializeClient());
  }

  void _registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'MAKE_REQUESTS', _makeInitialRequests);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'PLUGINS_INFO', _refreshTabsInfo);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'SERVER_READY', _setUpConfig);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'REQUEST_TAB', _requestTabFromServer);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CLOSE_TAB', _closeTabFromServer);
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
    _mailbox.ws.send('[[REQUEST_PLUGINSINFO]]');
    _mailbox.ws.send('[[CLIENT_CONFIG]]');
  }

  void _setUpConfig(Msg um) {
    _config = JSON.decode(um.body);
    _gotConfig.complete();
  }

  void _refreshTabsInfo(Msg um) {
    Map pluginsInfo = JSON.decode(um.body);
    _panelsInfo = pluginsInfo['panels'];
    _tabsInfo = pluginsInfo['tabs'];
    _gotPluginsInfo.complete();
  }

  void _requestTabFromServer(Msg um) {
    // Column is specified in the message body.
    if (um.body.contains(':')) {
      List<String> list = um.body.split(':');
      String refName = list[0];
      int col = int.parse(list[1]);

      int tabId = _getAvailableId(refName);
      _tabColumnControllers[col - 1].openTab(tabId, _tabsInfo[refName]);

      return;
    }

    int tabId = _getAvailableId(um.body);

    // A column wasn't specified, so the lowest Tab column that can add more tabs is used.
    for (TabColumnController controller in _tabColumnControllers) {
      if (controller.canAddMoreTabs) {
        controller.openTab(tabId, _tabsInfo[um.body]);
        break;
      }
    }
  }

  void _closeTabFromServer(Msg um) {
    String id = um.body;
    List idList = id.split(':');
    String refName = idList[0];
    int tabId = int.parse(idList[1]);

    for (TabColumnController controller in _tabColumnControllers) {
      // Break once one of the controllers finds the tab to close.
      if (controller.findAndCloseTab(tabId, refName)) break;
    }
  }

  void _moveTabFromServer(Msg um) {
    List idList = um.body.split(':');
    String refName = idList[0];
    int id = int.parse(idList[1]);

    // Working with indexes here, not the columnId.
    int oldColIndex = int.parse(idList[2]) - 1;
    int newColIndex = int.parse(idList[3]) - 1;

    // Don't go any further if a move request can't be done.
    if (!_tabColumnControllers[newColIndex].canAddMoreTabs) {
      window.alert('Can\'t move tab. Already at max on this side.');
      return;
    }

    PluginInterface tab = _tabColumnControllers[oldColIndex].removeTab(refName, id);
    _tabColumnControllers[newColIndex].addTab(tab);
  }

  void _issueAlert(Msg m) => window.alert(m.body);

  void _cleanUp(Msg m) {
    _panelColumnControllers.forEach((controller) => controller.cleanUp());
    _tabColumnControllers.forEach((controller) => controller.cleanUp());

    String alertMessage = 'UpDroid Commander has lost connection to the server.';
    alertMessage = alertMessage + ' Please restart the server (if necessary) and refresh the page.';
    window.alert(alertMessage);
  }

  //\/\/ Event Handlers /\/\/

  //\/\/ Misc Functions /\/\//

  /// Initializes all classes based on the loaded configuration in [_config].
  /// TODO: use isolates.
  void _initializeClient() {
    PanelColumnView.createPanelColumnView(0).then((view) {
      PanelColumnController controller = new PanelColumnController(0, view, _config[0], _mailbox, _panelsInfo, _tabIds);
      _panelColumnControllers.add(controller);

      controller.columnEvents.listen((ColumnEvent event) {
        if (event == ColumnEvent.LOST_FOCUS) {
          _panelColumnControllers.firstWhere((c) => c != controller).getsFocus();
        }
      });

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

      for (int i = 1; i < _config.length; i++) {
        // Start the Client with Column 1 maximized by default.
        ColumnState defaultState = i == 1 ? ColumnState.MAXIMIZED : ColumnState.MINIMIZED;

        TabColumnView.createTabColumnView(i, ColumnState.NORMAL).then((view) {
          TabColumnController controller = new TabColumnController(i, view, _config[i], _mailbox, _tabsInfo, _tabIds, ColumnState.NORMAL);
          _tabColumnControllers.add(controller);

          controller.columnStateChanges.listen((ColumnState newState) {
            if (newState == ColumnState.MAXIMIZED) {
              querySelector('body').style.minWidth = '770px';
              _tabColumnControllers.where((c) => c != controller).forEach((c) => c.minimize(false));
            } else if (newState == ColumnState.MINIMIZED) {
              querySelector('body').style.minWidth = '770px';
              _tabColumnControllers.where((c) => c != controller).forEach((c) => c.maximize(false));
            } else {
              querySelector('body').style.minWidth = '1211px';
              _tabColumnControllers.where((c) => c != controller).forEach((c) => c.resetToNormal(false));
            }
          });

          controller.columnEvents.listen((ColumnEvent event) {
            if (event == ColumnEvent.LOST_FOCUS) {
              _tabColumnControllers.firstWhere((c) => c != controller).getsFocus();
            }
          });
        });
      }

    });
  }

  int _getAvailableId(String refName) {
    if (!_tabIds.containsKey(refName)) _tabIds[refName] = [];

    // Find the lowest unused ID possible.
    int id = 0;
    bool found = false;
    while (!found) {
      id++;
      if (!_tabIds[refName].contains(id)) break;
    }

    // Add the new ID to the registry before handing it back out.
    _tabIds[refName].add(id);
    return id;
  }
}
