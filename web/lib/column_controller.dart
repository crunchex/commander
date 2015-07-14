library column_controller;

import 'dart:async';
import 'dart:html';

import 'tabs/tab_controller.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'column_view.dart';
import 'mailbox.dart';

enum ColumnState { MINIMIZED, NORMAL, MAXIMIZED }
enum ColumnEvent { LOST_FOCUS }

class ColumnController {
  int columnId;
  ColumnState state;
  Stream<ColumnState> columnStateChanges;
  Stream<ColumnEvent> columnEvents;

  StreamController<ColumnState> _columnStateChangesController;
  StreamController<ColumnEvent> _columnEventsController;

  List _config;
  Mailbox _mailbox;
  Function _getAvailableId;

  ColumnView _view;
  List<TabController> _tabs;

  ColumnController(this.columnId, this.state, List config, Mailbox mailbox, Function getAvailableId) {
    _config = config;
    _mailbox = mailbox;
    _getAvailableId = getAvailableId;

    _columnStateChangesController = new StreamController<ColumnState>();
    columnStateChanges = _columnStateChangesController.stream;
    _columnEventsController = new StreamController<ColumnEvent>();
    columnEvents = _columnEventsController.stream;
    _tabs = [];

    ColumnView.createColumnView(columnId, state).then((columnView) {
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
      if (!canAddMoreTabs) return;

      new UpDroidOpenTabModal(openTabFromModal);
    });

    _view.maximizeButton.onClick.listen((e) {
      if (state == ColumnState.NORMAL) {
        maximize(true);
      } else {
        resetToNormal(true);
      }
    });

    _view.columnContent.onKeyDown.listen((e) {
      if (!e.ctrlKey) return;

      // Cycle columns.
      if (e.shiftKey) {
        if ((e.keyCode == KeyCode.LEFT && columnId != 1) || (e.keyCode == KeyCode.RIGHT && columnId != 2)) {
          _columnEventsController.add(ColumnEvent.LOST_FOCUS);
        }

        return;
      }

      // Cycle tabs.
      TabController currentActiveTab = _tabs.firstWhere((TabController tab) => tab.view.tabHandle.classes.contains('active'));
      int currentActiveTabIndex = _tabs.indexOf(currentActiveTab);

      if (e.keyCode == KeyCode.LEFT && currentActiveTabIndex > 0) {
        cycleTab(true, currentActiveTab, currentActiveTabIndex);
      } else if (e.keyCode == KeyCode.RIGHT && currentActiveTabIndex < _tabs.length - 1) {
        cycleTab(false, currentActiveTab, currentActiveTabIndex);
      }
    });
  }

  void getsFocus() {
    _view.tabContent.querySelector('.active').children[1].focus();
  }

  void cycleTab(bool left, TabController currentActiveTab, int currentActiveTabIndex) {
    currentActiveTab.makeInactive();

    if (left) {
      _tabs[currentActiveTabIndex - 1].makeActive();
      _tabs[currentActiveTabIndex - 1].view.tabContent.focus();
    } else {
      _tabs[currentActiveTabIndex + 1].makeActive();
      _tabs[currentActiveTabIndex + 1].view.tabContent.focus();
    }
  }

  /// Maximizes the [ColumnController]'s state. If [external] == true, then an
  /// event is not fired to avoid an endless loop.
  void maximize(bool internal) {
    state = ColumnState.MAXIMIZED;
    if (internal) _columnStateChangesController.add(state);
    _view.maximize();
  }

  /// Resets the [ColumnController]'s state to normal. If [external] == true, then an
  /// event is not fired to avoid an endless loop.
  void resetToNormal(bool internal) {
    state = ColumnState.NORMAL;
    if (internal) _columnStateChangesController.add(state);
    _view.normalize();
  }

  /// Minimizes the [ColumnController]'s state. If [external] == true, then an
  /// event is not fired to avoid an endless loop.
  void minimize(bool internal) {
    state = ColumnState.MINIMIZED;
    if (internal) _columnStateChangesController.add(state);
    _view.minimize();
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
    if (!canAddMoreTabs) return;

    if (_tabs.isNotEmpty) {
      for (var tab in _tabs) {
        tab.makeInactive();
      }
    }

    TabController newTab;
    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      newTab = new UpDroidEditor(id, columnId);
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      newTab = new UpDroidCamera(id, columnId);
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      newTab = new UpDroidTeleop(id, columnId);
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      newTab = new UpDroidConsole(id, columnId);
    }

    _tabs.add(newTab);
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

  TabController removeTab(String tabType, int id) {
    TabController tab = _tabs.firstWhere((TabController t) => t.fullName == tabType && t.id == id);
    _tabs.remove(tab);

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((TabController tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    return tab;
  }

  void addTab(TabController tab) {
    // Make all tabs in that column inactive before adding the new one.
    _tabs.forEach((TabController tab) => tab.makeInactive());

    // Move the tab handle.
    querySelector('#column-${columnId.toString()}').children[1].children.add(tab.view.tabHandle);
    // Move the tab content.
    querySelector('#col-${columnId.toString()}-tab-content').children.add(tab.view.tabContainer);
    // Update the controller and view's columns.
    tab.col = columnId;
    tab.view.col = columnId;

    tab.makeActive();
    _tabs.add(tab);
  }

  bool get canAddMoreTabs => _tabs.length < _maxTabs;
  int get _maxTabs => (ColumnView.width[state] / 10 * 8).toInt();

  void cleanUp() {
    _columnStateChangesController.close();
    _columnEventsController.close();
  }
}