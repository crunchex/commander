library tab_interface;

import 'dart:html';

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';

import 'container_view.dart';
import 'tab_view.dart';
import 'panel_view.dart';
import 'launcher_view.dart';

class PluginInterface {
  int id, col;
  ContainerView view;
  Map pluginInfo;
  String refName, fullName, shortName;

  Mailbox _mailbox;
  ScriptElement _pluginJs;

  PluginInterface(this.id, this.col, this.pluginInfo, Mailbox mailbox, UListElement navTabs, DivElement columnContent, PluginType type, [bool asRequest=false]) {
    refName = pluginInfo['refName'];
    fullName = pluginInfo['fullName'];
    shortName = pluginInfo['shortName'];

    _mailbox = mailbox;

    _initiateTabSetup(type, navTabs, columnContent, asRequest);
  }

  bool isActive() => view.isActive();
  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();
  void shutdownScript() => _pluginJs.remove();

  void _initiateTabSetup(PluginType type, UListElement navTabs, DivElement columnContent, bool asRequest) {
    String pluginDir;
    switch (type) {
      case PluginType.LAUNCHER:
        view = new LauncherView(id, col, refName, fullName, shortName, navTabs, columnContent);
        pluginDir = 'tabs';
        break;
      case PluginType.TAB:
        view = new TabView(id, col, refName, fullName, shortName, navTabs, columnContent);
        pluginDir = 'tabs';
        break;
      case PluginType.PANEL:
        view = new PanelView(id, col, refName, fullName, shortName, navTabs, columnContent);
        pluginDir = 'panels';
        break;
    }

    // Launch the Tab's backend.
    String header = asRequest ? '[[OPEN_TAB_AS_REQUEST]]' : '[[OPEN_TAB]]';
    _mailbox.ws.send(header + '$col:$id:$refName');

    // Call the Tab's frontend (as a JS lib).
    _pluginJs = new ScriptElement()
    ..id = '$refName-$id-script'
    ..type = 'text/javascript'
    ..src = 'plugins/$refName/index.dart.js';

    document.body.children.add(_pluginJs);
  }
}