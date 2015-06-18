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

abstract class FileSystemEntityView {
  String name;
  LIElement element;
  DivElement container;
  SpanElement icon;

  FileSystemEntityView(this.name) {
    element = new LIElement()
      ..classes.add('explorer-li');

    container = new DivElement();
    element.children.add(container);

    icon = new SpanElement()
      ..classes.add('glyphicons');
    container.children.add(icon);

    SpanElement filename = new SpanElement()
      ..classes.add('explorer-fs-name')
      ..text = this.name;
    container.children.add(filename);
  }
}

class FolderView extends FileSystemEntityView {
  final String openFolderClass = 'glyphicons-folder-open';
  final String closedFolderClass = 'glyphicons-folder-closed';

  bool expanded;
  UListElement uElement;

  FolderView(String name, [bool expanded=false]) : super(name) {
    this.expanded = expanded;

    container.classes.add('explorer-folder');
    this.expanded ? icon.classes.add(openFolderClass) : icon.classes.add(closedFolderClass);

    uElement = new UListElement()
      ..classes.add('explorer-ul');
    element.children.add(uElement);
  }

  void toggleExpansion() {
    if (expanded) {
      icon.classes.remove(openFolderClass);
      icon.classes.add(closedFolderClass);
      expanded = false;
    } else {
      icon.classes.remove(closedFolderClass);
      icon.classes.add(openFolderClass);
      expanded = true;
    }
  }
}

class FileView extends FileSystemEntityView {
  final String fileClass = 'glyphicons-file';

  FileView(String name) : super(name) {
    container.classes.add('explorer-file');
    icon.classes.add(fileClass);
  }
}