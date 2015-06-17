part of updroid_explorer;

class ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<ExplorerView> createExplorerView(int id, String name, DivElement content) {
    Completer c = new Completer();
    c.complete(new ExplorerView(id, name, content));
    return c.future;
  }

  DivElement content;
  DivElement explorersDiv;
  SpanElement folder;
  SpanElement file;
  DivElement drop;

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

  ExplorerView(int id, String name, DivElement content) {
    this.content = content;

    _dropdown = querySelector('#exp-dropdown');
    _titleWrap = querySelector('#title-wrapper');
    explorersDiv = querySelector("#exp-container");
    _expList = querySelector("#side-menu ul");
    _controlPanel = querySelector('#control');
    _packageList = querySelector('#packages');
    _title = querySelector('#file-explorer-title');
    _controlToggle = querySelector('#control-toggle');
    _recycle = querySelector('#recycle');

    explorersDiv = new DivElement()
      ..id = 'exp-container';
    content.children.add(explorersDiv);

    _explorer = new DivElement()
      ..id = "exp-$id"
      ..classes.add('exp')
      ..dataset['num'] = id.toString()
      ..dataset['name'] = name;
    explorersDiv.children.add(_explorer);

    _recycle = new ParagraphElement()
      ..id = 'recycle'
      ..text = 'Recycle ';
    explorersDiv.children.add(_recycle);

    SpanElement trash = new SpanElement()
      ..id = 'trash'
      ..classes.addAll(['glyphicons', 'glyphicons-trash']);
    _recycle.children.add(trash);

    DivElement explorerHead = new DivElement()
      ..classes.add('explorer-head');
    _explorer.append(explorerHead);

    _hrContainer = new DivElement()
      ..id = "file-explorer-hr-container-$id";
    explorerHead.append(_hrContainer);

    drop = new DivElement()
      ..classes.add("new-file-drop")
      ..id = "new-file-drop-$id";
    _hrContainer.append(drop);

    DivElement body = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container'])
      ..id = "explorer-$id";
    _explorer.append(body);

    UListElement guts = new UListElement()
      ..classes.add("explorer-body")
      ..id = "explorer-body-$id";
    body.append(guts);
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