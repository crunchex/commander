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

  ExplorerView() {
    separator = querySelector('#side-menu-separator');

    _explorersDiv = querySelector("#exp-container");
    _expList = querySelector("#side-menu ul");
    _controlPanel = querySelector('#control');
    _title = querySelector('#file-explorer-title');
    _controlToggle = querySelector('#control-toggle');
    _recycle = querySelector('#recycle');
  }

  Map createExplorer(int num, String name) {

    Map explorerRefs = {'explorersDiv' : _explorersDiv, 'control' : _controlPanel, 'title' : _title, 'controlToggle' : _controlToggle, 'recycle' : _recycle};
    ParagraphElement recycle = querySelector("#recycle");
    makeExpButton(num, name);
    _explorer = new DivElement()
      ..id = "exp-$num"
      ..dataset['num'] = num.toString();
    _explorersDiv.insertBefore(_explorer, recycle);
    explorerRefs['explorer'] = _explorer;

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
    SpanElement folder = new SpanElement()
      ..id = "folder-$num"
      ..classes.addAll(["glyphicon glyphicon-folder-close", 'folder']);
    SpanElement file = new SpanElement()
      ..id = "file-$num"
      ..classes.addAll(["glyphicon glyphicon-file", 'file']);
    explorerRefs['file'] = file;
    explorerRefs['folder'] = folder;
    newDnd.append(folder);
    newDnd.append(file);
    DivElement hrContainer = new DivElement()
      ..id = "file-explorer-hr-container-$num";
    explorerHead.append(hrContainer);
    explorerRefs['hrContainer'] = hrContainer;
    DivElement drop = new DivElement()
      ..classes.add("new-file-drop")
      ..id = "new-file-drop-$num";
    hrContainer.append(drop);
    explorerRefs['drop'] = drop;
    ParagraphElement p = new ParagraphElement();
    p.text = "Top Level";
    drop.append(p);
    DivElement body = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container'])
      ..id = "explorer-$num";
    _explorer.append(body);
    explorerRefs['expBody'] = body;
    UListElement guts = new UListElement()
      ..classes.add("explorer-body")
      ..id = "explorer-body-$num";
    body.append(guts);
    explorerRefs['expList'] = guts;
    return explorerRefs;
  }

  makeExpButton (int num, name) {
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
        if(explorer.id != 'recycle' && explorer.id != 'control-buttons') {
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

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExlorer() {
    _explorer.classes.remove('hidden');
  }
}