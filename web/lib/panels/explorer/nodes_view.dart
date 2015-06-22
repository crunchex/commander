part of updroid_explorer;

class NodesView extends ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<NodesView> createNodesView(int id, DivElement content) {
    Completer c = new Completer();
    c.complete(new NodesView(id, content));
    return c.future;
  }

  UListElement uList;

  NodesView(int id, DivElement content) :
  super(id, content) {
    this.content = content;

    DivElement explorerContainer = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container']);
//    explorersDiv.append(explorerContainer);

    uList = new UListElement()
      ..classes.add("explorer-ul");
    explorersDiv.append(uList);

    DivElement toolbar = new DivElement()
      ..classes.add('toolbar');
    explorersDiv.children.add(toolbar);
  }

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExplorer() {
    _explorer.classes.remove('hidden');
  }

  void cleanUp() {
    content.innerHtml = '';
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
  }
}

abstract class RosEntityView {
  final String selectedClass = 'selected';

  String name;
  LIElement element;
  DivElement container;
  SpanElement icon, filename;

  bool _selected = false;

  RosEntityView(this.name) {
    element = new LIElement()
      ..classes.add('explorer-li');

    container = new DivElement()
      ..classes.add('explorer-ros-container')
      ..style.userSelect = 'none';
    element.children.add(container);

    icon = new SpanElement()
      ..classes.addAll(['glyphicons', 'explorer-icon']);
    container.children.add(icon);

    filename = new SpanElement()
      ..classes.add('explorer-ros-name')
      ..text = this.name;
    container.children.add(filename);
  }

  InputElement startRename() {
    InputElement renameInput = new InputElement()
      ..classes.add('explorer-fs-rename')
      ..placeholder = name;
    filename.replaceWith(renameInput);

    renameInput.onClick.first.then((e) => e.stopPropagation());
    renameInput.focus();
    return renameInput;
  }

  void completeRename(InputElement renameInput) {
    renameInput.replaceWith(filename);
  }

  void select() {
    container.classes.add(selectedClass);
    _selected = true;
  }

  void deselect() {
    container.classes.remove(selectedClass);
    _selected = false;
  }

  void cleanUp() {
    for (Element child in element.children) {
      child.remove();
    }
    element.remove();
  }
}

class PackageView extends RosEntityView {
  final String openFolderClass = 'glyphicons-package';
  final String closedFolderClass = 'glyphicons-cargo';

  bool expanded = false;
  UListElement uElement;

  PackageView(String name, [bool expanded]) : super(name) {
    this.expanded = expanded;

    container.classes.add('explorer-package');
    this.expanded ? icon.classes.add(openFolderClass) : icon.classes.add(closedFolderClass);

    uElement = new UListElement()
      ..hidden = true
      ..classes.add('explorer-ul');
    element.children.add(uElement);
  }

  void toggleExpansion() {
    if (expanded) {
      icon.classes.remove(openFolderClass);
      icon.classes.add(closedFolderClass);
      uElement.hidden = true;
      expanded = false;
    } else {
      icon.classes.remove(closedFolderClass);
      icon.classes.add(openFolderClass);
      uElement.hidden = false;
      expanded = true;
    }
  }
}

class NodeView extends RosEntityView {
  final String fileClass = 'glyphicons-play-button';

  NodeView(String name) : super(name) {
    container.classes.add('explorer-node');
    icon.classes.add(fileClass);
  }
}