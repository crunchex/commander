library column_controller;

import 'dart:async';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';

import 'column_view.dart';

enum ColumnState { MINIMIZED, NORMAL, MAXIMIZED }
enum ColumnEvent { LOST_FOCUS }

abstract class ColumnController {
  int columnId;
  ColumnState state;
  Stream<ColumnState> columnStateChanges;
  Stream<ColumnEvent> columnEvents;
  Map pluginInfo;
  Map<String, List<int>> tabIds;

  StreamController<ColumnState> columnStateChangesController;
  StreamController<ColumnEvent> _columnEventsController;

  List config;
  Mailbox mailbox;
  bool disableCyclingHotkeys;

  ColumnView view;

  ColumnController(this.columnId, this.view, this.config, this.mailbox, this.pluginInfo, this.tabIds) {
    columnStateChangesController = new StreamController<ColumnState>();
    columnStateChanges = columnStateChangesController.stream;

    _columnEventsController = new StreamController<ColumnEvent>();
    columnEvents = _columnEventsController.stream;

    // This controls whether or not we listen to hotkeys for tab and column switching.
    // TODO: figure out some good hotkeys to use.
    disableCyclingHotkeys = true;

    _setUp();
  }

  void setUpController();
  void registerEventHandlers();

  void _setUp() {
    // General [ColumnController] controller setup.
    _setUpController();

    // Controller setup implemented by child class.
    setUpController();

    // General [ColumnController] Event Handlers.
    _registerEventHandlers();

    // Extra Event Handlers implemented by child class.
    registerEventHandlers();
  }

  void _setUpController() {

  }

  void _registerEventHandlers() {
    // If we haven't enabled cycling, don't set up the folowing event handler.
    if (disableCyclingHotkeys) return;
    view.columnContent.onKeyDown.listen((e) {
      if (!e.ctrlKey) return;

      // Cycle columns.
      if (e.shiftKey) {
        if ((e.keyCode == KeyCode.PAGE_DOWN && columnId != 1) || (e.keyCode == KeyCode.PAGE_UP && columnId != 2)) {
          _columnEventsController.add(ColumnEvent.LOST_FOCUS);
        }
      }
    });
  }

  int getAvailableId(String refName) {
    if (!tabIds.containsKey(refName)) tabIds[refName] = [];

    // Find the lowest unused ID possible.
    int id = 0;
    bool found = false;
    while (!found) {
      id++;
      if (!tabIds[refName].contains(id)) break;
    }

    // Add the new ID to the registry before handing it back out.
    tabIds[refName].add(id);
    print('tabIds in column_controller: ${tabIds.toString()}');
    return id;
  }

  void getsFocus() {
    view.tabContent.querySelector('.active').children[1].focus();
  }

  void cleanUp() {
    view.cleanUp();
    columnStateChangesController.close();
    _columnEventsController.close();
  }
}