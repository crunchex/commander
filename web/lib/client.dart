library updroid_client;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'tabs/tab_controller.dart';
import 'panels/panel_controller.dart';
import 'panels/explorer/explorer.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'mailbox.dart';
import 'column_view.dart';

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
    const bootStrapColumnsForTabs = 10;
    const numberOfPanels = 1;
    int columnWidth = bootStrapColumnsForTabs ~/ (config.length - numberOfPanels);

    for (int i = 1; i < config.length; i++) {
      _columnControllers.add(new ColumnController(i, columnWidth, config[i], _mailbox, _getAvailableId));
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

class ColumnController {
  int columnId;
  int width;
  
  List _config;
  Mailbox _mailbox;
  Function _getAvailableId;

  ColumnView _view;
  List _tabs;

  ColumnController(this.columnId, this.width, List config, Mailbox mailbox, Function getAvailableId) {
    _config = config;
    _mailbox = mailbox;
    _getAvailableId = getAvailableId;
    
    _tabs = [];

    ColumnView.createColumnView(columnId, this.width).then((columnView) {
      _view = columnView;

      setUpController();
      registerEventHandlers();
    });
  }

  void setUpController() {
    for (Map tab in _config) {
      openTab(tab['id'], tab['class']);
    }
  }

  void registerEventHandlers() {
    _view.controlButton.onClick.listen((e) {
      e.preventDefault();
      if (!_canAddMoreTabs) return;

      new UpDroidOpenTabModal(openTabFromModal);
    });
  }

  /// Returns a list of IDs of all tabs whose type match [className].
  List<int> returnIds(String className) {
    List<int> ids = [];
    _tabs.forEach((tab) {
      if (tab.fullName == className) ids.add(tab.id);
    });
    return ids;
  }

  /// Opens a [TabController].
  void openTab(int id, String className) {
    if (!_canAddMoreTabs) return;

    if (_tabs.isNotEmpty) {
      for (var tab in _tabs) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidEditor(id, columnId));
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidCamera(id, columnId));
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidTeleop(id, columnId));
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _tabs.add(new UpDroidConsole(id, columnId));
    }
  }

  /// A wrapper for [openTab] when an availble ID needs to be chosen across all open [ColumnController]s.
  void openTabFromModal(String className) {
    int id = _getAvailableId(className);
    openTab(id, className);
  }

  /// Locates a [TabController] by [tabId] and [tabType] and closes it. Returns true if
  /// said tab was found.
  bool findAndCloseTab(int tabId, String tabType) {
    bool found = false;
    for (int i = 0; i < _tabs.length; i++) {
      if (_tabs[i].fullName == tabType && _tabs[i].id == tabId) {
        found = true;
        _tabs.removeAt(i);
        break;
      }
    }

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((TabController tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + tabType + '_' + tabId.toString());

    return found;
  }

  bool get _canAddMoreTabs => _tabs.length < _maxTabs;
  int get _maxTabs => (width / 10 * 8).toInt();
}
