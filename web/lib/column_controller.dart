library column_controller;

import 'tabs/tab_controller.dart';
import 'tabs/teleop/teleop.dart';
import 'tabs/editor/editor.dart';
import 'tabs/console/console.dart';
import 'tabs/camera/camera.dart';
import 'modal/modal.dart';
import 'column_view.dart';
import 'mailbox.dart';

class ColumnController {
  int columnId;
  int width;

  List _config;
  Mailbox _mailbox;
  Function _getAvailableId;

  ColumnView _view;
  List _tabs;

  ColumnController(this.columnId, this.width, List config, Mailbox mailbox, Function getAvailableId) {
    _config = config;
    _mailbox = mailbox;
    _getAvailableId = getAvailableId;

    _tabs = [];

    ColumnView.createColumnView(columnId, this.width).then((columnView) {
      _view = columnView;

      setUpController();
      registerEventHandlers();
    });
  }

  void setUpController() {
    for (Map tab in _config) {
      openTab(tab['id'], tab['class']);
    }
  }

  void registerEventHandlers() {
    _view.controlButton.onClick.listen((e) {
      e.preventDefault();
      if (!_canAddMoreTabs) return;

      new UpDroidOpenTabModal(openTabFromModal);
    });
  }

  /// Returns a list of IDs of all tabs whose type match [className].
  List<int> returnIds(String className) {
    List<int> ids = [];
    _tabs.forEach((tab) {
      if (tab.fullName == className) ids.add(tab.id);
    });
    return ids;
  }

  /// Opens a [TabController].
  void openTab(int id, String className) {
    if (!_canAddMoreTabs) return;

    if (_tabs.isNotEmpty) {
      for (var tab in _tabs) {
        tab.makeInactive();
      }
    }

    if (className == 'UpDroidEditor') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidEditor(id, columnId));
    } else if (className == 'UpDroidCamera') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidCamera(id, columnId));
    } else if (className == 'UpDroidTeleop') {
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className');
      _tabs.add(new UpDroidTeleop(id, columnId));
    } else if (className == 'UpDroidConsole') {
      // TODO: initial size should not be hardcoded.
      _mailbox.ws.send('[[OPEN_TAB]]' + '$columnId-$id-$className-25-80');
      //Isolate console = await spawnDomUri(new Uri.file('lib/tabs/console.dart'), ['test'], [id, column, true]);
      _tabs.add(new UpDroidConsole(id, columnId));
    }
  }

  /// A wrapper for [openTab] when an availble ID needs to be chosen across all open [ColumnController]s.
  void openTabFromModal(String className) {
    int id = _getAvailableId(className);
    openTab(id, className);
  }

  /// Locates a [TabController] by [tabId] and [tabType] and closes it. Returns true if
  /// said tab was found.
  bool findAndCloseTab(int tabId, String tabType) {
    bool found = false;
    for (int i = 0; i < _tabs.length; i++) {
      if (_tabs[i].fullName == tabType && _tabs[i].id == tabId) {
        found = true;
        _tabs.removeAt(i);
        break;
      }
    }

    // Make all tabs in that column inactive except the last.
    _tabs.forEach((TabController tab) => tab.makeInactive());

    // If there's at least one tab left, make the last one active.
    if (_tabs.isNotEmpty) _tabs.last.makeActive();

    _mailbox.ws.send('[[CLOSE_TAB]]' + tabType + '_' + tabId.toString());

    return found;
  }

  bool get _canAddMoreTabs => _tabs.length < _maxTabs;
  int get _maxTabs => (width / 10 * 8).toInt();
}