library tab_controller;

import 'dart:html';
import 'dart:async';

import '../mailbox.dart';
import '../container_view.dart';
import '../context_menu.dart';

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

      view.tabHandleButton.onContextMenu.listen((e) {
        e.preventDefault();
        List menu = [
          {'type': 'toggle', 'title': 'Clone', 'handler': _cloneTab},
          {'type': 'toggle', 'title': 'Move ${col == 1 ? 'Right' : 'Left'}', 'handler': () => _moveTabTo(col == 1 ? 2 : 1)}];
        ContextMenu.createContextMenu(e.page, menu);
      });

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

  void _cloneTab() => mailbox.ws.send('[[CLONE_TAB]]' + '${fullName}_${id}_$col');
  void _moveTabTo(int newCol) => mailbox.ws.send('[[MOVE_TAB]]' + '${fullName}_${id}_${col}_$newCol');
}