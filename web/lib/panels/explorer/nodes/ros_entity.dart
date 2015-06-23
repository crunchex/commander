part of updroid_explorer_nodes;

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
  var deselectAllNodes;

  WebSocket _ws;

  bool _selectEnabled, _selected;

  Node(this.name, this.args, this.packageName, WebSocket ws, this.deselectAllNodes) {
    _ws = ws;
    _selected = false;
    _selectEnabled = true;

    _setUpNodeView();

    //print('workspacePath: $workspacePath, path: $path, name: $name, parent: $parent');
  }

  void runNode() {
    if (!_selected) return;

    List runCommand = [];
    runCommand.addAll([packageName, name]);
    view.uElement.children.forEach((LIElement li) {
      List<String> arg = [];

      DivElement container = li.firstChild;
      arg.add(container.firstChild.text);
      InputElement input = container.lastChild;
      arg.add(input.value);

      runCommand.add(arg);
    });

    _ws.send('[[RUN_NODE]]' + JSON.encode(runCommand));
  }


  void _setUpNodeView() {
    view = new NodeView(name, args);

    view.container.onClick.listen((e) {
      if (_selectEnabled) {
        if (!e.ctrlKey) deselectAllNodes();

        toggleSelected();
        _selectEnabled = false;

        new Timer(new Duration(milliseconds: 500), () {
          _selectEnabled = true;
        });
      }
    });

    view.container.onContextMenu.listen((e) {
      e.preventDefault();
      deselectAllNodes();
      select();
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