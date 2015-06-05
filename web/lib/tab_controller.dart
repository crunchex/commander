library tab_controller;

import 'dart:html';
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

  AnchorElement _closeTabButton;

  TabController(this.id, this.col, this.tabType, this.shortName, List menuConfig, [StreamController<CommanderMessage> cs]) {
    if (cs == null) {
      mailbox = new Mailbox(tabType, id);
    } else {
      this.cs = cs;
      mailbox = new Mailbox(tabType, id, this.cs);
    }
    registerMailbox();

    TabView.createTabView(id, col, tabType, shortName, menuConfig).then((tabView) {
      view = tabView;

      _closeTabButton = view.refMap['close-tab'];
      _closeTabButton.onClick.listen((e) => _closeTab());
      view.closeControlHitbox.onClick.listen((e) => _closeTab());
      view.cloneControlHitbox.onClick.listen((e) => _cloneTab(e));

      setUpController();
      registerEventHandlers();
    });
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();

  void registerMailbox();
  void setUpController();
  void registerEventHandlers();
  void cleanUp();

  void _closeTab() {
    view.destroy();
    cleanUp();
    cs.add(new CommanderMessage('UPDROIDCLIENT', 'CLOSE_TAB', body: '${tabType}_$id'));
  }

  void _cloneTab(Event e) {
    e.preventDefault();
    cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${col}_${tabType}'));
  }
}