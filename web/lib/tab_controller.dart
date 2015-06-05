library tab_controller;

import 'dart:async';

import 'mailbox.dart';
import 'updroid_message.dart';
import 'tab_view.dart';

abstract class TabController {
  int id;
  int col;
  StreamController<CommanderMessage> cs;
  bool active;
  String tabType;

  TabView view;
  Mailbox mailbox;

  TabController(this.id, this.col, this.tabType, this.cs) {
    mailbox = new Mailbox(tabType, id, cs);
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();

  List _getMenuConfig();
}