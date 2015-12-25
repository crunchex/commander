library tab_column_controller;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';

import 'tab_column_view.dart';
import 'column_controller.dart';
import 'plugin_interface.dart';
import 'container_view.dart';

class TabColumnController extends ColumnController {
  List<PluginInterface> _tabs = [];

  ColumnState state;

  TabColumnController(int columnId, TabColumnView view, List config, Mailbox mailbox, Map tabInfo, Map tabIds, this.state) :
  super(columnId, view, config, mailbox, tabInfo, tabIds);

  void setUpController() {
    // Always open a launcher tab first.
    int id = getAvailableId('upcom-launcher');
    Map launcherInfo = {'refName': 'upcom-launcher', 'fullName': 'UpDroid Launcher', 'shortName': 'Launcher'};
    openTab(id, launcherInfo);

    for (Map tab in config) {
      int id = tab['id'];
      String refName = tab['class'];

      if (!tabIds.containsKey(refName)) tabIds[refName] = [];
      tabIds[refName].add(tab['id']);
      openTab(id, pluginInfo[refName]);
    }
  }

  void registerEventHandlers() {
    TabColumnView tabColumnView = view;

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
      PluginInterface currentActiveTab = _tabs.firstWhere((PluginInterface tab) => tab.isActive());
      int currentActiveTabIndex = _tabs.indexOf(currentActiveTab);

      if (e.keyCode == KeyCode.PAGE_DOWN && currentActiveTabIndex > 0) {
        cycleTab(true, currentActiveTab, currentActiveTabIndex);
      } else if (e.keyCode == KeyCode.PAGE_UP && currentActiveTabIndex < _tabs.length - 1) {
        cycleTab(false, currentActiveTab, currentActiveTabIndex);
      }
    });
  }

  void cycleTab(bool left, PluginInterface currentActiveTab, int currentActiveTabIndex) {
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
    if (!canAddMoreTabs) {
      // "Cancel opening" by removing the ID that was just added to the registry.
      // TODO: fix the following corner case:
      // where we could end up with an apparent "ID skip" if two of the same tabs were
      // being opened at almost the same time and the lower one was cancelled.
      tabIds[tabInfo['refName']].remove(id);
      return null;
    }

    if (_tabs.isNotEmpty) {
      for (PluginInterface tab in _tabs) {
        tab.makeInactive();
      }
    }

    PluginInterface tab;
    if (asRequest) {
      tab = new PluginInterface(id, columnId, tabInfo, mailbox, view.navTabs, view.tabContent, PluginType.TAB, true);
    } else {
      tab = new PluginInterface(id, columnId, tabInfo, mailbox, view.navTabs, view.tabContent, PluginType.TAB);
    }

    _tabs.add(tab);
    return null;
  }

  /// Locates a [TabController] by [tabId] and [refName] and closes it. Returns true if
  /// said tab was found.
  bool findAndCloseTab(int tabId, String refName) {
    bool found = false;
    for (int i = 0; i < _tabs.length; i++) {
      if (_tabs[i].refName == refName && _tabs[i].id == tabId) {
        found = true;

        tabIds[refName].remove(_tabs[i].id);
        _tabs[i].view.destroy();
        _tabs[i].shutdownScript();
        _tabs.removeAt(i);
        break;
      }
    }

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((PluginInterface tab) => tab.makeInactive());
    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    return found;
  }

  PluginInterface removeTab(String refName, int id) {
    PluginInterface tab = _tabs.firstWhere((PluginInterface t) => t.refName == refName && t.id == id);
    _tabs.remove(tab);

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((PluginInterface tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    return tab;
  }

  void addTab(PluginInterface tab) {
    // Make all tabs in that column inactive before adding the new one.
    _tabs.forEach((PluginInterface tab) => tab.makeInactive());

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