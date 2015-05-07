library tab_controller;

import 'dart:async';

import 'mailbox.dart';
import 'updroid_message.dart';
import 'tab_view.dart';

class TabController {
  int id;
  int col;
  StreamController<CommanderMessage> cs;
  bool active;
  String tabType;

  TabView view;
  Mailbox mailbox;

  TabController(this.id, this.col, this.tabType, this.cs, {bool active: false}) {
    if (active != null) this.active = active;

    mailbox = new Mailbox(tabType, id, cs);
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
}