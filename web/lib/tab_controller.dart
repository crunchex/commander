library tab_controller;

import 'dart:async';

import 'mailbox.dart';
import 'updroid_message.dart';
import 'tab_view.dart';

abstract class TabController {
  int id, col;
  StreamController<CommanderMessage> cs;
  bool active;
  String tabType, shortName;

  TabView view;
  Mailbox mailbox;

  TabController(this.id, this.col, this.tabType, this.shortName, List menuConfig, [StreamController<CommanderMessage> cs]) {
    if (cs == null) {
      mailbox = new Mailbox(tabType, id);
    } else {
      this.cs = cs;
      mailbox = new Mailbox(tabType, id, this.cs);
    }

    TabView.createTabView(id, col, tabType, shortName, menuConfig).then((tabView) {
      view = tabView;
      setUpController();
    });
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();

  void setUpController();
}