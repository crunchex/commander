part of updroid_finder;

abstract class FinderView {
  Element separator = querySelector('#side-menu-separator');

  DivElement _explorersDiv;
  DivElement _titleWrap;
  UListElement _expList;
  DivElement _controlPanel;
  LIElement _title;
  ButtonElement _controlToggle;
  ParagraphElement _recycle;
  DivElement _explorer;
  DivElement _hrContainer;
  SpanElement _folder;
  SpanElement _file;
  DivElement _drop;
  UListElement _packageList;
  ButtonElement _dropdown;

  Future createExplorer(int num, String name) {
    Completer completer = new Completer();

    separator = querySelector('#side-menu-separator');

    _dropdown = querySelector('#exp-dropdown');
    _titleWrap = querySelector('#title-wrapper');
    _explorersDiv = querySelector("#exp-container");
    _expList = querySelector("#side-menu ul");
    _controlPanel = querySelector('#control');
    _packageList = querySelector('#packages');
    _title = querySelector('#file-explorer-title');
    _controlToggle = querySelector('#control-toggle');
    _recycle = querySelector('#recycle');

    ParagraphElement recycle = querySelector("#recycle");
    makeExpButton(num, name);
    _explorer = new DivElement()
      ..id = "exp-$num"
      ..classes.add('exp')
      ..dataset['num'] = num.toString()
      ..dataset['name'] = name;
    _explorersDiv.insertBefore(_explorer, recycle);

    DivElement explorerHead = new DivElement()
      ..classes.add('explorer-head');
    _explorer.append(explorerHead);
    ParagraphElement folderName = new ParagraphElement()
      ..classes.add('workspaceName')
      ..text = name;
    explorerHead.append(folderName);
    LIElement newDnd = new LIElement()
      ..classes.add('new')
      ..text = "New";
    explorerHead.append(newDnd);
    _folder = new SpanElement()
      ..id = "folder-$num"
      ..classes.addAll(['glyphicons', 'glyphicons-folder-closed', 'folder']);
    _file = new SpanElement()
      ..id = "file-$num"
      ..classes.addAll(['glyphicons', 'glyphicons-file', 'file']);
    newDnd.append(_folder);
    newDnd.append(_file);
    _hrContainer = new DivElement()
      ..id = "file-explorer-hr-container-$num";
    explorerHead.append(_hrContainer);
    _drop = new DivElement()
      ..classes.add("new-file-drop")
      ..id = "new-file-drop-$num";
    _hrContainer.append(_drop);
    ParagraphElement p = new ParagraphElement();
    p.text = "Top Level";
    _drop.append(p);
    DivElement body = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container'])
      ..id = "explorer-$num";
    _explorer.append(body);
    UListElement guts = new UListElement()
      ..classes.add("explorer-body")
      ..id = "explorer-body-$num";
    body.append(guts);

    completer.complete();
    return completer.future;
  }

  makeExpButton (int num, String name) {
    LIElement item = new LIElement()
      ..id = "exp-li-$num";
    AnchorElement link = new AnchorElement()
      ..href = "#"
      ..text = name
      ..attributes['role'] = 'button';
    item.append(link);
    _expList.insertBefore(item, separator);
    item.onClick.listen((e){
      if(_explorersDiv.classes.contains('hidden')) {
        _explorersDiv.classes.remove('hidden');
        _controlPanel.classes.add('hidden');
        _controlToggle.classes.add('shadow');
        _dropdown.classes.add('shadow');
        _titleWrap.classes.remove('shadow');
      }
      for(var explorer in _explorersDiv.children) {
        if(explorer.id != 'recycle' && !explorer.classes.contains('control-buttons')) {
          if(!explorer.classes.contains('hidden') && int.parse(explorer.dataset['num']) != num) {
            explorer.classes.add('hidden');
          }
          if(int.parse(explorer.dataset['num']) == num) {
            explorer.classes.remove('hidden');
          }
        }
      }
    });
  }

  ///Create Node List

  Map createPackageList (List<Map> nodeList) {
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