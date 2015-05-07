part of updroid_explorer;

abstract class ExplorerView {
  Element separator = querySelector('#side-menu-separator');

  DivElement _explorersDiv;
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
  UListElement _nodeList;

  Future createExplorer(int num, String name) {
    Completer completer = new Completer();

    separator = querySelector('#side-menu-separator');

    _explorersDiv = querySelector("#exp-container");
    _expList = querySelector("#side-menu ul");
    _controlPanel = querySelector('#control');
    _nodeList = querySelector('#nodes ul');
    _title = querySelector('#file-explorer-title');
    _controlToggle = querySelector('#control-toggle');
    _recycle = querySelector('#recycle');

    ParagraphElement recycle = querySelector("#recycle");
    makeExpButton(num, name);
    _explorer = new DivElement()
      ..id = "exp-$num"
      ..dataset['num'] = num.toString();
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
    LIElement item = new LIElement();
    AnchorElement link = new AnchorElement()
      ..id = "exp-button-$num"
      ..href = "#"
      ..text = name
      ..attributes['role'] = 'button';
    item.append(link);
    _expList.insertBefore(item, separator);
    item.onClick.listen((e){
      if(_explorersDiv.classes.contains('hidden')) {
        _explorersDiv.classes.remove('hidden');
        _controlPanel.classes.add('hidden');
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

  createNodeLi(Map packageNode) {
    String _nodeName = packageNode['node'];
    LIElement node = new LIElement()
      ..classes.add('node')
      ..dataset['name'] = _nodeName
      ..text = _nodeName;
    _nodeList.append(node);
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
    }

    _nodeList.append(nodeArgs);

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