part of updroid_explorer;

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

      _mailbox.ws.send('[[CATKIN_NODE_LIST]]');

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

class Package {
  String name, path;
  List<Node> nodes;
  PackageView view;

  Package(this.name, this.path) {
    nodes = [];

    setUpPackageView();
  }

  void setUpPackageView() {
    view = new PackageView(name);

    view.container.onDoubleClick.listen((e) {
      view.toggleExpansion();
    });
  }

  void cleanUp() {
    nodes.forEach((Node n) => n.cleanUp());
    view.cleanUp();
  }
}

class Node {
  String name, packageName;
  List args;
  NodeView view;

  WebSocket _ws;

  bool _selectEnabled, _selected;

  Node(this.name, this.args, this.packageName, WebSocket ws) {
    _ws = ws;
    _selected = false;
    _selectEnabled = true;

    _setUpNodeView();

    //print('workspacePath: $workspacePath, path: $path, name: $name, parent: $parent');
  }

  void runNode() {
    List runCommand = [];
    runCommand.addAll([packageName, name]);
    view.uElement.children.forEach((LIElement li) {
      List<String> arg = [];

      DivElement container = li.firstChild;
      arg.add(container.firstChild.text);
      arg.add(container.lastChild.value);

      runCommand.add(arg);
    });

    _ws.send('[[CATKIN_RUN]]' + JSON.encode(runCommand));
  }


  void _setUpNodeView() {
    view = new NodeView(name, args);

    view.container.onClick.listen((e) {
      if (_selectEnabled) {
        toggleSelected();
        _selectEnabled = false;

        new Timer(new Duration(milliseconds: 500), () {
          _selectEnabled = true;
        });
      }
    });

    view.container.onContextMenu.listen((e) {
      e.preventDefault();
      List menu = [
        {'type': 'toggle', 'title': 'Run', 'handler': runNode}];
      ContextMenu.createContextMenu(e.page, menu);
    });
  }

  void toggleSelected() => _selected ? deselect() : select();

  void select() {
    view.select();
    _selected = true;
  }

  void deselect() {
    view.deselect();
    _selected = false;
  }

  void cleanUp() {
    //_contextListeners.forEach((StreamSubscription listener) => listener.cancel());
    view.cleanUp();
  }
}