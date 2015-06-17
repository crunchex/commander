part of updroid_explorer;

abstract class ExplorerView {
  DivElement content;
  DivElement explorersDiv;
  SpanElement folder;
  SpanElement file;
  DivElement drop;

  DivElement _titleWrap;
  DivElement _controlPanel;
  LIElement _title;
  ButtonElement _controlToggle;
  DivElement _explorer;
  DivElement _hrContainer;
  UListElement _packageList;
  ButtonElement _dropdown;

  ExplorerView(int id, DivElement content) {
    this.content = content;

    explorersDiv = new DivElement()
      ..id = 'exp-container';
    content.children.add(explorersDiv);

    _explorer = new DivElement()
      ..id = "exp-$id"
      ..classes.add('exp')
      ..dataset['num'] = id.toString();
    explorersDiv.children.add(_explorer);

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
  }
}