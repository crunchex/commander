library tab_interface;

import 'dart:async';
import 'dart:convert';
import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';

import 'container_view.dart';
import 'tab_view.dart';
import 'panel_view.dart';
import 'launcher_view.dart';

class TabInterface {
  int id, col;
  ContainerView view;
  Map tabInfo;
  String refName, fullName, shortName;

  Mailbox _mailbox;
  ScriptElement _tabJs;

  TabInterface(this.id, this.col, this.tabInfo, Mailbox mailbox, UListElement navTabs, DivElement columnContent, PluginType type, [bool asRequest=false]) {
    refName = tabInfo['refName'];
    fullName = tabInfo['fullName'];
    shortName = tabInfo['shortName'];

    _mailbox = mailbox;

    _initiateTabSetup(type, navTabs, columnContent, asRequest);
  }

  bool isActive() => view.isActive();
  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
  void shutdownScript() => _tabJs.remove();

  void _initiateTabSetup(PluginType type, UListElement navTabs, DivElement columnContent, bool asRequest) {
    switch (type) {
      case PluginType.LAUNCHER:
        view = new LauncherView(id, col, refName, fullName, shortName, navTabs, columnContent);
        break;
      case PluginType.TAB:
        view = new TabView(id, col, refName, fullName, shortName, navTabs, columnContent);
        break;
      case PluginType.PANEL:
        view = new PanelView(id, col, refName, fullName, shortName, navTabs, columnContent);
        break;
    }

    // Launch the Tab's backend.
    String header = asRequest ? '[[OPEN_TAB_AS_REQUEST]]' : '[[OPEN_TAB]]';
    _mailbox.ws.send(header + '$col:$id:$refName');

    // Call the Tab's frontend (as a JS lib).
    _tabJs = new ScriptElement()
    ..id = '$refName-$id-script'
    ..type = 'text/javascript'
    ..src = 'tabs/$refName/index.dart.js';

    document.body.children.add(_tabJs);
  }
}