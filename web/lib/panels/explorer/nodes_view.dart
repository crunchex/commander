part of updroid_explorer;

class NodesView extends ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<NodesView> createNodesView(int id, String name, DivElement content) {
    Completer c = new Completer();
    c.complete(new NodesView(id, name, content));
    return c.future;
  }

  DivElement content;
  DivElement explorersDiv;
  SpanElement folder;
  SpanElement file;
  DivElement drop;
  SpanElement trash;

  DivElement _titleWrap;
  UListElement _expList;
  DivElement _controlPanel;
  LIElement _title;
  ButtonElement _controlToggle;
  ParagraphElement _recycle;
  DivElement _explorer;
  DivElement _hrContainer;
  UListElement _packageList;
  ButtonElement _dropdown;

  NodesView(int id, String name, DivElement content) :
  super(id, name, content) {
    this.content = content;

  }

  ///Create Node List

  Map createPackageList(List<Map> nodeList) {
    List packages = [];
    Map packageList = {};
    for (var item in nodeList) {
      if (!packages.contains(item['package'])) packages.add(item['package']);
    }
    _packageList.innerHtml = '';
    for(var package in packages) {
      DivElement packageWrap = new DivElement();
      _packageList.append(packageWrap);
      LIElement title = new LIElement()
        ..classes.add('package-title');
      packageWrap.append(title);
      SpanElement icon = new SpanElement()
        ..classes.addAll(['glyphicons', 'glyphicons-cargo']);
      SpanElement packageName = new SpanElement()
        ..classes.add('package-name')
        ..text = package;
      title.append(icon);
      title.append(packageName);
      UListElement packageFiles = new UListElement()
        ..classes.add('package-files');
      // map of ulist items to append launch files and nodes
      packageList[package] = packageFiles;
      packageWrap.append(packageFiles);
    }

    return packageList;
  }

  LIElement createNodeLi(StreamController<CommanderMessage> cs, Map packageNode) {
    String _fileName = packageNode['node'];
    LIElement packageFile = new LIElement()
      ..dataset['name'] = _fileName
      ..dataset['package'] = packageNode['package']
      ..dataset['package-path'] = packageNode['package-path'];
    if (packageNode['node'].contains('.launch')) {
      SpanElement launch = new SpanElement();
      launch.classes.addAll(['glyphicons', 'glyphicons-send']);
      packageFile.append(launch);
      packageFile.classes.add('launch');
    }
    else {
      SpanElement node = new SpanElement();
      node.classes.addAll(['glyphicons', 'glyphicons-collapse']);
      packageFile.append(node);
      packageFile.classes.add('node');
    }
    SpanElement nodeLaunch = new SpanElement();
    var shortened = _fileName.replaceAll('.launch', '');
    shortened = shortened.replaceAll('.py', '');
    nodeLaunch.text = shortened;
    packageFile.append(nodeLaunch);

    InputElement nodeArgs = new InputElement()
      ..classes.add('node-args-input')
      ..classes.add('hidden');

    if (packageNode.containsKey('args')) {
      String arguments = '';
      packageNode['args'].forEach((List arg) {
        if (arg.length == 1) {
          arguments += '${arg[0]}:=';
        } else {
          arguments += ' ${arg[0]}:=${arg[1]}';
        }
      });
      nodeArgs.placeholder = arguments;

      nodeArgs.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          cs.add(new CommanderMessage('EXPLORER', 'RUN_NODE'));
        }
      });

      packageFile.append(nodeArgs);
    }

    return packageFile;

//    ButtonElement nodeButton = _createButton('default', buttonText, method: () {
//      String runCommand;
//      if (nodeArgs.value.isEmpty) {
//        runCommand = JSON.encode([packageNode['package'], packageNode['package-path'], packageNode['node']]);
//      } else {
//        runCommand = JSON.encode([packageNode['package'], packageNode['package-path'], packageNode['node'], nodeArgs.value]);
//      }
//      //_ws.send('[[CATKIN_RUN]]' + runCommand);
//      _cs.add(new CommanderMessage('EXPLORER', 'CATKIN_RUN', body: runCommand));
//    });
//    nodeButton
//      ..dataset['toggle'] = 'tooltip'
//      ..dataset['placement'] = 'bottom'
//      ..title = nodeName;
//    new Tooltip(nodeButton, showDelay: 700, container: _nodeList);
//    nodeWrap.children.add(nodeButton);
//    nodeWrap.children.add(nodeArgs);
  }

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExplorer() {
    _explorer.classes.remove('hidden');
  }
}