library panel_column_controller;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';

import 'panel_column_view.dart';
import 'column_controller.dart';
import 'panel_interface.dart';

class PanelColumnController extends ColumnController {
  List<PanelInterface> _panels = [];

  PanelColumnView panelColumnView;

  PanelColumnController(int columnId, ColumnState state, List config, Mailbox mailbox, Map panelInfo, Function getAvailableId) :
  super(columnId, state, config, mailbox, panelInfo, getAvailableId, (columnId) => PanelColumnView.createPanelColumnView(columnId)) {
    panelColumnView = view;
  }

  Future setUpController() async {
    for (Map panel in config) {
      await openPanel(panel['id'], pluginInfo[panel['class']]);
    }
  }

  void registerEventHandlers() {
    // If we haven't enabled cycling, don't set up the folowing event handler.
    if (disableCyclingHotkeys) return;
    view.columnContent.onKeyDown.listen((e) {
      if (!e.ctrlKey && ! e.shiftKey) return;

      // Cycle panels.
      PanelInterface currentActivePanel = _panels.firstWhere((PanelInterface panel) => panel.isActive());
      int currentActivePanelIndex = _panels.indexOf(currentActivePanel);

      if (e.keyCode == KeyCode.PAGE_DOWN && currentActivePanelIndex > 0) {
        cyclePanel(true, currentActivePanel, currentActivePanelIndex);
      } else if (e.keyCode == KeyCode.PAGE_UP && currentActivePanelIndex < _panels.length - 1) {
        cyclePanel(false, currentActivePanel, currentActivePanelIndex);
      }
    });
  }

  void cyclePanel(bool left, PanelInterface currentActivePanel, int currentActivePanelIndex) {
    currentActivePanel.makeInactive();

    if (left) {
      _panels[currentActivePanelIndex - 1].makeActive();
      _panels[currentActivePanelIndex - 1].view.tabContent.focus();
    } else {
      _panels[currentActivePanelIndex + 1].makeActive();
      _panels[currentActivePanelIndex + 1].view.tabContent.focus();
    }
  }

  /// Returns a list of IDs of all panels whose type match [refName].
  List<int> returnIds(String refName) {
    List<int> ids = [];
    _panels.forEach((panel) {
      if (panel.refName == refName) ids.add(panel.id);
    });
    return ids;
  }

  /// Opens a [PanelController].
  Future openPanel(int id, Map panelInfo) async {
    if (!canAddMorepanels) return null;

    if (_panels.isNotEmpty) {
      for (PanelInterface panel in _panels) {
        panel.makeInactive();
      }
    }

    PanelInterface panel = new PanelInterface(id, columnId, panelInfo, mailbox);
    await panel.setupComplete;
    _panels.add(panel);

    return null;
  }

  /// A wrapper for [openPanel] when an availble ID needs to be chosen across all open [ColumnController]s.
  void openPanelFromModal(Map panelInfo) {
    int id = getAvailableId(panelInfo['refName']);
    openPanel(id, panelInfo);
  }

  /// Locates a [PanelController] by [panelId] and [refName] and closes it. Returns true if
  /// said panel was found.
  bool findAndClosePanel(int panelId, String refName) {
    bool found = false;
    for (int i = 0; i < _panels.length; i++) {
      if (_panels[i].refName == refName && _panels[i].id == panelId) {
        found = true;
        _panels[i].shutdownScript();
        _panels.removeAt(i);
        break;
      }
    }

    // Make all panels in that column inactive except the last.
    _panels.forEach((PanelInterface panel) => panel.makeInactive());

    // If there's at least one panel left, make the last one active.
    if (_panels.isNotEmpty) _panels.last.makeActive();

    mailbox.ws.send('[[CLOSE_PANEL]]' + refName + ':' + panelId.toString());

    return found;
  }

  PanelInterface removePanel(String refName, int id) {
    PanelInterface panel = _panels.firstWhere((PanelInterface t) => t.refName == refName && t.id == id);
    _panels.remove(panel);

    // Make all panels in that column inactive except the last.
    _panels.forEach((PanelInterface panel) => panel.makeInactive());

    // If there's at least one panel left, make the last one active.
    if (_panels.isNotEmpty) _panels.last.makeActive();

    return panel;
  }

  void addPanel(PanelInterface panel) {
    // Make all panels in that column inactive before adding the new one.
    _panels.forEach((PanelInterface panel) => panel.makeInactive());

    // Move the tab handle.
    querySelector('#column-${columnId.toString()}').children[1].children.add(panel.view.tabHandle);
    // Move the tab content.
    querySelector('#col-${columnId.toString()}-tab-content').children.add(panel.view.tabContainer);

    // Send a message to update the column on the real classes.
    mailbox.ws.send('[[UPDATE_COLUMN]]' + panel.refName + ':' + panel.id.toString() + ':' + columnId.toString());

    // Update the [PanelInterface] and [PanelViewInterface]'s columns.
    panel.col = columnId;
    panel.view.col = columnId;

    panel.makeActive();
    _panels.add(panel);
  }

  bool get canAddMorepanels => _panels.length < 1;
}