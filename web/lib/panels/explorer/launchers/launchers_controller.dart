library updroid_explorer_launchers;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'package:path/path.dart' as pathLib;

import '../../../context_menu.dart';
import '../../../mailbox.dart';
import '../../panel_controller.dart';
import '../explorer.dart';

part 'launchers_view.dart';
part 'ros_entity.dart';

class LaunchersController implements ExplorerController {
  PanelView _view;
  LaunchersView _launchersView;
  Mailbox _mailbox;

  AnchorElement _runLaunchersButton;

  Map<String, Package> packages = {};
  String workspacePath;
  Set<StreamSubscription> _listenersToCleanUp;

  LaunchersController(int id, this.workspacePath, PanelView view, Mailbox mailbox, List<AnchorElement> actionButtons) {
    _view = view;
    _mailbox = mailbox;
    _runLaunchersButton = actionButtons[0];

    registerMailbox();

    _listenersToCleanUp = new Set<StreamSubscription>();

    LaunchersView.createLaunchersView(id, _view.content).then((launchersView) {
      _launchersView = launchersView;

      _mailbox.ws.send('[[REQUEST_NODE_LIST]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'LAUNCH', addLaunch);
  }

  void registerEventHandlers() {
    _listenersToCleanUp.add(_runLaunchersButton.onClick.listen((e) {
      packages.values.forEach((Package p) => p.launchers.forEach((Launcher n) => n.runLauncher()));
    }));
  }

  void addLaunch(UpDroidMessage um) {
    Map data = JSON.decode(um.body);
    String packagePath = data['package-path'];
    String launcherName = data['node'];
    List args = data['args'];

    if (!packages.containsKey(packagePath)) _addPackage(packagePath);

    String packageName = packagePath.split('/').last;
    Launcher launcher = new Launcher(launcherName, args, packageName, _mailbox.ws, _deselectAllLaunchers);
    packages[packagePath].launchers.add(launcher);
    packages[packagePath].view.uElement.children.add(launcher.view.element);
  }

  void _addPackage(String packagePath) {
    List<String> split = packagePath.split('/');
    String parentPath = '/';
    parentPath += pathLib.joinAll(split.sublist(0, split.length - 1));

    if (parentPath != '$workspacePath/src' && !packages.containsKey(parentPath)) {
      _addPackage(parentPath);
    }

    String packageName = split.last;
    Package package = new Package(packageName, packagePath);
    packages.putIfAbsent(packagePath, () => package);

    if (parentPath == '$workspacePath/src') {
      _launchersView.uList.children.add(package.view.element);
    } else {
      packages[parentPath].view.uElement.children.add(package.view.element);
    }

  }

  void _deselectAllLaunchers() {
    packages.values.forEach((Package p) => p.launchers.forEach((Launcher n) => n.deselect()));
  }

  void cleanUp() {
    _listenersToCleanUp.forEach((StreamSubscription s) => s.cancel());

    packages.values.forEach((Package p) => p.cleanUp());
    packages = null;

    _launchersView.cleanUp();
  }
}