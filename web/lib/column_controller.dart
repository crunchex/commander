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

  StreamController<ColumnState> columnStateChangesController;
  StreamController<ColumnEvent> _columnEventsController;

  List config;
  Mailbox mailbox;
  Function getAvailableId;
  Function viewStaticConstructor;
  bool disableCyclingHotkeys;

  ColumnView view;

  ColumnController(this.columnId, this.config, this.mailbox, this.pluginInfo, this.getAvailableId, this.viewStaticConstructor, [ColumnState state]) {
    columnStateChangesController = new StreamController<ColumnState>();
    columnStateChanges = columnStateChangesController.stream;

    _columnEventsController = new StreamController<ColumnEvent>();
    columnEvents = _columnEventsController.stream;

    // This controls whether or not we listen to hotkeys for tab and column switching.
    // TODO: figure out some good hotkeys to use.
    disableCyclingHotkeys = true;

    if (state == null) {
      viewStaticConstructor(columnId).then((columnView) => _postViewSetupCallback(columnView));
    } else {
      this.state = state;
      viewStaticConstructor(columnId, this.state).then((columnView) => _postViewSetupCallback(columnView));
    }
  }

  Future setUpController();
  void registerEventHandlers();

  void _postViewSetupCallback(ColumnView columnView) {
    view = columnView;

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

  void getsFocus() {
    view.tabContent.querySelector('.active').children[1].focus();
  }

  void cleanUp() {
    view.cleanUp();
    columnStateChangesController.close();
    _columnEventsController.close();
  }
}