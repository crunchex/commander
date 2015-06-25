library updroid_explorer_nodes;

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

class UpDroidNodes implements ExplorerController {
  PanelView _view;
  NodesView _nodesView;
  Mailbox _mailbox;

  AnchorElement _runNodesButton;

  Map<String, Package> packages = {};
  String workspacePath;
  Set<StreamSubscription> _listenersToCleanUp;

  UpDroidNodes(int id, this.workspacePath, PanelView view, Mailbox mailbox, List<AnchorElement> actionButtons) {
    _view = view;
    _mailbox = mailbox;
    _runNodesButton = actionButtons[0];

    registerMailbox();

    _listenersToCleanUp = new Set<StreamSubscription>();

    NodesView.createNodesView(id, _view.content).then((nodesView) {
      _nodesView = nodesView;

      _mailbox.ws.send('[[REQUEST_NODE_LIST]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'LAUNCH', addLaunch);
  }

  void registerEventHandlers() {
    _listenersToCleanUp.add(_runNodesButton.onClick.listen((e) {
      packages.values.forEach((Package p) => p.nodes.forEach((Node n) => n.runNode()));
    }));
  }

  void addLaunch(UpDroidMessage um) {
    Map data = JSON.decode(um.body);
    String packagePath = data['package-path'];
    String nodeName = data['node'];
    List args = data['args'];

    if (!packages.containsKey(packagePath)) _addPackage(packagePath);

    String packageName = packagePath.split('/').last;
    Node node = new Node(nodeName, args, packageName, _mailbox.ws, _deselectAllNodes);
    packages[packagePath].nodes.add(node);
    packages[packagePath].view.uElement.children.add(node.view.element);
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
      _nodesView.uList.children.add(package.view.element);
    } else {
      packages[parentPath].view.uElement.children.add(package.view.element);
    }

  }

  void _deselectAllNodes() {
    packages.values.forEach((Package p) => p.nodes.forEach((Node n) => n.deselect()));
  }

  void cleanUp() {
    _listenersToCleanUp.forEach((StreamSubscription s) => s.cancel());

    packages.values.forEach((Package p) => p.cleanUp());
    packages = null;

    _nodesView.cleanUp();
  }
}