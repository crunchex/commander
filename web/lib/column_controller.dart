library column_controller;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/modal/modal.dart';
import 'package:upcom-api/web/mailbox/mailbox.dart';

import 'column_view.dart';
import 'tab_interface.dart';

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
  bool _disableCyclingHotkeys;

  ColumnView _view;
  List<TabInterface> _tabs;

  ColumnController(this.columnId, this.state, List config, Mailbox mailbox, Function getAvailableId) {
    _config = config;
    _mailbox = mailbox;
    _getAvailableId = getAvailableId;

    _columnStateChangesController = new StreamController<ColumnState>();
    columnStateChanges = _columnStateChangesController.stream;
    _columnEventsController = new StreamController<ColumnEvent>();
    columnEvents = _columnEventsController.stream;
    _tabs = [];

    // This controls whether or not we listen to hotkeys for tab and column switching.
    // TODO: figure out some good hotkeys to use.
    _disableCyclingHotkeys = true;

    ColumnView.createColumnView(columnId, state).then((columnView) {
      _view = columnView;

      setUpController();
      registerEventHandlers();
    });
  }

  Future setUpController() async {
    for (Map tab in _config) {
      await openTab(tab['id'], tab['class']);
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

    if (_disableCyclingHotkeys) return;
    _view.columnContent.onKeyDown.listen((e) {
      if (!e.ctrlKey) return;

      // Cycle columns.
      if (e.shiftKey) {
        if ((e.keyCode == KeyCode.PAGE_DOWN && columnId != 1) || (e.keyCode == KeyCode.PAGE_UP && columnId != 2)) {
          _columnEventsController.add(ColumnEvent.LOST_FOCUS);
        }

        return;
      }

      // Cycle tabs.
      TabInterface currentActiveTab = _tabs.firstWhere((TabInterface tab) => tab.isActive());
      int currentActiveTabIndex = _tabs.indexOf(currentActiveTab);

      if (e.keyCode == KeyCode.PAGE_DOWN && currentActiveTabIndex > 0) {
        cycleTab(true, currentActiveTab, currentActiveTabIndex);
      } else if (e.keyCode == KeyCode.PAGE_UP && currentActiveTabIndex < _tabs.length - 1) {
        cycleTab(false, currentActiveTab, currentActiveTabIndex);
      }
    });
  }

  void getsFocus() {
    _view.tabContent.querySelector('.active').children[1].focus();
  }

  void cycleTab(bool left, TabInterface currentActiveTab, int currentActiveTabIndex) {
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
  Future openTab(int id, String className) async {
    if (!canAddMoreTabs) return null;

    if (_tabs.isNotEmpty) {
      for (TabInterface tab in _tabs) {
        tab.makeInactive();
      }
    }

    TabInterface tab = new TabInterface(id, columnId, className, _mailbox);
    await tab.setupComplete;
    _tabs.add(tab);

    return null;
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
        _tabs[i].shutdownScript();
        _tabs.removeAt(i);
        break;
      }
    }

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((TabInterface tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + tabType + '_' + tabId.toString());

    return found;
  }

  TabInterface removeTab(String tabType, int id) {
    TabInterface tab = _tabs.firstWhere((TabInterface t) => t.fullName == tabType && t.id == id);
    _tabs.remove(tab);

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((TabInterface tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    return tab;
  }

  void addTab(TabInterface tab) {
    // Make all tabs in that column inactive before adding the new one.
    _tabs.forEach((TabInterface tab) => tab.makeInactive());

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