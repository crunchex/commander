library tab_column_controller;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/modal/modal.dart';

import 'tab_column_view.dart';
import 'column_controller.dart';
import 'tab_interface.dart';

class TabColumnController extends ColumnController {
  List<TabInterface> _tabs = [];
  var i = 0;

  TabColumnController(int columnId, ColumnState state, List config, Mailbox mailbox, Map tabInfo, Function getAvailableId) :
  super(columnId, config, mailbox, tabInfo, getAvailableId, TabColumnView.createTabColumnView, state) {

  }

  Future setUpController() async {
    for (Map tab in config) {
      await openTab(tab['id'], pluginInfo[tab['class']]);
    }
  }

  void registerEventHandlers() {
    TabColumnView tabColumnView = view;
    tabColumnView.controlButton.onClick.listen((e) {
      e.preventDefault();
      if (!canAddMoreTabs) return;
      mailbox.ws.send('[[REQUEST_PLUGINSINFO]]');
      new UpDroidOpenTabModal(openTabFromModal, pluginInfo);
    });

    tabColumnView.maximizeButton.onClick.listen((e) {
      if (state == ColumnState.NORMAL) {
        maximize(true);
      } else {
        resetToNormal(true);
      }
    });

    // If we haven't enabled cycling, don't set up the folowing event handler.
    if (disableCyclingHotkeys) return;
    tabColumnView.columnContent.onKeyDown.listen((e) {
      if (!e.ctrlKey && ! e.shiftKey) return;

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
    if (internal) columnStateChangesController.add(state);
    TabColumnView tabColumnView = view;
    tabColumnView.maximize();
  }

  /// Resets the [ColumnController]'s state to normal. If [external] == true, then an
  /// event is not fired to avoid an endless loop.
  void resetToNormal(bool internal) {
    state = ColumnState.NORMAL;
    if (internal) columnStateChangesController.add(state);
    TabColumnView tabColumnView = view;
    tabColumnView.normalize();
  }

  /// Minimizes the [ColumnController]'s state. If [external] == true, then an
  /// event is not fired to avoid an endless loop.
  void minimize(bool internal) {
    state = ColumnState.MINIMIZED;
    if (internal) columnStateChangesController.add(state);
    TabColumnView tabColumnView = view;
    tabColumnView.minimize();
  }

  /// Returns a list of IDs of all tabs whose type match [refName].
  List<int> returnIds(String refName) {
    List<int> ids = [];
    _tabs.forEach((tab) {
      if (tab.refName == refName) ids.add(tab.id);
    });
    return ids;
  }

  /// Opens a [TabController].
  Future openTab(int id, Map tabInfo, [bool asRequest=false]) async {
    if (!canAddMoreTabs) return null;

    if (_tabs.isNotEmpty) {
      for (TabInterface tab in _tabs) {
        tab.makeInactive();
      }
    }

    TabInterface tab;
    if (asRequest) {
      tab = new TabInterface(id, columnId, tabInfo, mailbox, true);
    } else {
      tab = new TabInterface(id, columnId, tabInfo, mailbox);
    }

    await tab.setupComplete;
    _tabs.add(tab);

    return null;
  }

  /// A wrapper for [openTab] when an availble ID needs to be chosen across all open [ColumnController]s.
  void openTabFromModal(Map tabInfo) {
    int id = getAvailableId(tabInfo['refName']);
    openTab(id, tabInfo);
  }

  /// Locates a [TabController] by [tabId] and [refName] and closes it. Returns true if
  /// said tab was found.
  bool findAndCloseTab(int tabId, String refName) {
    bool found = false;
    for (int i = 0; i < _tabs.length; i++) {
      if (_tabs[i].refName == refName && _tabs[i].id == tabId) {
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

    mailbox.ws.send('[[CLOSE_TAB]]' + refName + ':' + tabId.toString());

    return found;
  }

  TabInterface removeTab(String refName, int id) {
    TabInterface tab = _tabs.firstWhere((TabInterface t) => t.refName == refName && t.id == id);
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
    querySelector('#column-${columnId.toString()}').children[0].children.add(tab.view.tabHandle);
    // Move the tab content.
    querySelector('#col-${columnId.toString()}-tab-content').children.add(tab.view.tabContainer);

    // Send a message to update the column on the real classes.
    mailbox.ws.send('[[UPDATE_COLUMN]]' + tab.refName + ':' + tab.id.toString() + ':' + columnId.toString());

    // Update the [TabInterface] and [TabViewInterface]'s columns.
    tab.col = columnId;
    tab.view.col = columnId;

    tab.makeActive();
    _tabs.add(tab);
  }

  bool get canAddMoreTabs => _tabs.length < _maxTabs;
  int get _maxTabs => (TabColumnView.width[state] / 10 * 8).toInt();
}