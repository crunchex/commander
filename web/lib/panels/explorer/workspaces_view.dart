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

  SpanElement trash;
  ParagraphElement _recycle;

  WorkspacesView(int id, DivElement content) :
  super(id, content) {
    this.content = content;

    DivElement body = new DivElement()
      ..classes.addAll(['well', 'well-sm', 'explorer-container'])
      ..id = "explorer-$id";
    _explorer.append(body);

    UListElement guts = new UListElement()
      ..classes.add("explorer-body")
      ..id = "explorer-body-$id";
    body.append(guts);

    _recycle = new ParagraphElement()
      ..id = 'recycle';
    explorersDiv.children.add(_recycle);

    trash = new SpanElement()
      ..id = 'trash'
      ..classes.addAll(['glyphicons', 'glyphicons-bin']);
    _recycle.children.add(trash);
  }

  void hideExplorer() {
    _explorer.classes.add('hidden');
  }

  void showExplorer() {
    _explorer.classes.remove('hidden');
  }
}