part of updroid_explorer;

class UpDroidNodes implements ExplorerController {
  PanelView _view;
  NodesView _nodesView;
  Mailbox _mailbox;

  AnchorElement _runButton;

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
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CATKIN_NODE_LIST', populateNodes);
  }

  void registerEventHandlers() {

  }

  void populateNodes(UpDroidMessage um) {
    List<Map> nodeList = JSON.decode(um.body);
    Map packageMap = _nodesView.createPackageList(nodeList);

    for (var packageNode in nodeList) {
      if(!packageNode['node'].contains('.xml')) {
        var element = _nodesView.createNodeLi(cs, packageNode);
        var listToAppend = packageMap[packageNode['package']];
        listToAppend.append(element);
        setupNodeHighlighter(element);
      }
    }
  }

  void setupNodeHighlighter (LIElement li) {
    li.onClick.listen((e) {
      if(currentSelectedNode != null) {
        currentSelectedNode.classes.remove('highlighted');
        nodeArgs.classes.add('hidden');
      }
      li.classes.add('highlighted');
      currentSelectedNode = li;
      nodeArgs = li.lastChild;
      runParams.addAll({'name' : li.dataset['name'], 'package' : li.dataset['package'], 'package-path' : li.dataset['package-path']});
      nodeArgs.classes.remove('hidden');
    });
  }

  void _catkinNodeList(CommanderMessage m) => mailbox.ws.send('[[CATKIN_NODE_LIST]]');

  void _runNode() {
    String runCommand;
    if (nodeArgs.value.isEmpty) {
      runCommand = JSON.encode([runParams['package'], runParams['package-path'], runParams['name']]);
    } else {
      runCommand = JSON.encode([runParams['package'], runParams['package-path'], runParams['name'], nodeArgs.value]);
    }
    mailbox.ws.send('[[CATKIN_RUN]]' + runCommand);
  }

  void cleanUp() {

  }
}