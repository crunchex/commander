library tab_controller;

import 'dart:async';

import 'mailbox.dart';
import 'updroid_message.dart';
import 'tab_view.dart';

class TabController {
  int num;
  int col;
  StreamController<CommanderMessage> cs;

  TabView view;
  Mailbox mailbox;

  TabController(this.num, int col, String className, StreamController<CommanderMessage> cs, {bool active: false}) {
    col = col;
    cs = cs;

    mailbox = new Mailbox(className, num, cs);
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
}