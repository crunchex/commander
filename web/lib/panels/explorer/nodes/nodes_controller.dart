library updroid_explorer_nodes;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import '../../../context_menu.dart';
import '../../../mailbox.dart';
import '../../panel_controller.dart';
import '../explorer.dart';

part 'nodes_view.dart';
part 'ros_entity.dart';

class UpDroidNodes implements ExplorerController {
  PanelView _view;
  NodesView _nodesView;
  Mailbox _mailbox;

  AnchorElement _runButton;

  Map<String, Package> packages = {};
  String workspacePath;

  UpDroidNodes(int id, this.workspacePath, PanelView view, Mailbox mailbox) {
    _view = view;
    _mailbox = mailbox;

    registerMailbox();

    NodesView.createNodesView(id, _view.content).then((nodesView) {
      _nodesView = nodesView;

      _runButton = _view.refMap['run'];

      _mailbox.ws.send('[[REQUEST_NODE_LIST]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'LAUNCH', addLaunch);
  }

  void registerEventHandlers() {

  }

  void addLaunch(UpDroidMessage um) {
    Map data = JSON.decode(um.body);
    String packageName = data['package'];
    String packagePath = data['package-path'];
    String nodeName = data['node'];
    List args = data['args'];

    Package package = packages.putIfAbsent(packageName, () => new Package(packageName, packagePath));
    if (package != null) _nodesView.uList.children.add(package.view.element);

    Node node = new Node(nodeName, args, packageName, _mailbox.ws);
    packages[packageName].nodes.add(node);
    packages[packageName].view.element.children.add(node.view.element);
  }

  void cleanUp() {
    packages.values.forEach((Package p) => p.cleanUp());
    packages = null;
    _nodesView.cleanUp();
  }
}