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

  FileSystemEntityView(this.name) {
    element = new LIElement()
      ..classes.add('explorer-li');
  }
}

class FolderView extends FileSystemEntityView {
  final String openFolderClass = 'glyphicons-folder-open';
  final String closedFolderClass = 'glyphicons-folder-closed';

  bool expanded;
  UListElement uElement;

  DivElement _container;
  SpanElement _icon, _fileName;

  FolderView(String name, [bool expanded=false]) : super(name) {
    this.expanded = expanded;

    _container = new DivElement()
      ..id = this.name.toLowerCase();
//    setupHighlighter(fileContainer);
    _container.classes.addAll(['explorer-fs-container', 'explorer-folder']);
    element.children.add(_container);

    // Create a span element for the glyphicon
    _icon = new SpanElement()
      ..classes.add('glyphicons');
    this.expanded ? _icon.classes.add(openFolderClass) : _icon.classes.add(closedFolderClass);
    _container.children.add(_icon);
//    dropSetup(glyphicon, file);
//    dropSetup(glyph, file);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement()
      ..classes.add('explorer-fs-name')
      ..text = this.name;
    _container.children.add(filename);

    uElement = new UListElement()
      ..classes.add('explorer-ul');
    element.children.add(uElement);
  }

  void toggleExpansion() {
    if (expanded) {
      _icon.classes.remove(openFolderClass);
      _icon.classes.add(closedFolderClass);
      expanded = false;
    } else {
      _icon.classes.remove(closedFolderClass);
      _icon.classes.add(openFolderClass);
      expanded = true;
    }
  }
}

class FileView extends FileSystemEntityView {
  final String fileClass = 'glyphicons-file';

  DivElement _container;
  SpanElement _icon, _fileName;

  FileView(String name) : super(name) {
    _container = new DivElement()
      ..id = this.name.toLowerCase();
//    setupHighlighter(fileContainer);
    _container.classes.addAll(['explorer-fs-container', 'explorer-file']);
    element.children.add(_container);

    // Create a span element for the glyphicon
    _icon = new SpanElement()
      ..classes.addAll(['glyphicons', fileClass]);
    _container.children.add(_icon);
//    dropSetup(glyphicon, file);
//    dropSetup(glyph, file);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement()
      ..classes.add('explorer-fs-name')
      ..text = this.name;
    _container.children.add(filename);
  }
}