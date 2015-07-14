library tab_controller;

import 'dart:html';
import 'dart:async';

import '../mailbox.dart';
import '../container_view.dart';

part 'tab_view.dart';

abstract class TabController {
  int id, col;
  bool active;
  String fullName, shortName;

  TabView view;
  Mailbox mailbox;

  AnchorElement _closeTabButton;

  TabController(this.id, this.col, this.fullName, this.shortName, List menuConfig, [bool externalCss=false]) {
    mailbox = new Mailbox(fullName, id);

    registerMailbox();

    TabView.createTabView(id, col, fullName, shortName, menuConfig, externalCss).then((tabView) {
      view = tabView;

      _closeTabButton = view.refMap['close-tab'];
      _closeTabButton.onClick.listen((e) => _closeTab());
      view.closeControlHitbox.onClick.listen((e) => _closeTab());

      setUpController();
      registerEventHandlers();

      // When the content of this tab receives focus, transfer it to whatever is the main content of the tab
      // (which may or may not be the direct child of view.content).
      // Also, this is done last as additional view set up may have been done in setUpController().
      view.tabContent.onFocus.listen((e) => elementToFocus.focus());
    });
  }

  void makeActive() => view.makeActive();
  void makeInactive() => view.makeInactive();

  void registerMailbox();
  void setUpController();
  void registerEventHandlers();
  void onFocus();
  Future<bool> preClose();
  void cleanUp();
  Element get elementToFocus;

  Future _closeTab() async {
    // Cancel closing if preClose returns false for some reason.
    bool canClose = await preClose();
    if (!canClose) return new Future.value(true);

    view.destroy();
    cleanUp();

    UpDroidMessage um = new UpDroidMessage('CLOSE_TAB', '${fullName}_$id');
    mailbox.ws.send(um.s);
  }

//  void _cloneTab(Event e) {
//    e.preventDefault();
//    if (cs != null) {
//      cs.add(new CommanderMessage('UPDROIDCLIENT', 'OPEN_TAB', body: '${col}_${tabType}'));
//    } else {
//      UpDroidMessage um = new UpDroidMessage('CLOSE_TAB', '${tabType}_$id');
//      mailbox.ws.send(um.s);
//    }
//  }
}