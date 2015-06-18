part of updroid_explorer;

class WorkspacesView extends ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<WorkspacesView> createWorkspacesView(int id, DivElement content) {
    Completer c = new Completer();
    c.complete(new WorkspacesView(id, content));
    return c.future;
  }

  SpanElement recycle;
  UListElement uList;

  WorkspacesView(int id, DivElement content) :
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

    recycle = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-bin', 'recycle']);
    toolbar.children.add(recycle);
  }

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExplorer() {
    _explorer.classes.remove('hidden');
  }
}