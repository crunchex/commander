part of updroid_explorer_workspace;

class WorkspaceView extends ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<WorkspaceView> createWorkspaceView(int id, DivElement content) {
    Completer c = new Completer();
    c.complete(new WorkspaceView(id, content));
    return c.future;
  }

  SpanElement recycle;
  UListElement uList;

  WorkspaceView(int id, DivElement content) :
  super(id, content) {
    this.content = content;

    uList = new UListElement()
      ..classes.add("explorer-ul");
    explorersDiv.append(uList);

    DivElement toolbar = new DivElement()
      ..classes.add('toolbar');
    explorersDiv.children.add(toolbar);

    recycle = new SpanElement()
      ..classes.addAll(['glyphicons', 'glyphicons-bin', 'recycle']);
//    toolbar.children.add(recycle);
  }

  void cleanUp() {
    content.innerHtml = '';
  }
}

abstract class FileSystemEntityView {
  final String selectedClass = 'selected';

  String name;
  LIElement element;
  DivElement container;
  SpanElement icon, filename;

  bool _selected = false;

  FileSystemEntityView(this.name) {
    element = new LIElement()
      ..classes.add('explorer-li');

    container = new DivElement()
      ..classes.add('explorer-fs-container')
      ..style.userSelect = 'none';
    element.children.add(container);

    icon = new SpanElement()
      ..classes.addAll(['glyphicons', 'explorer-icon']);
    container.children.add(icon);

    filename = new SpanElement()
      ..classes.add('explorer-fs-name')
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

class FolderView extends FileSystemEntityView {
  final String openFolderClass = 'glyphicons-folder-open';
  final String closedFolderClass = 'glyphicons-folder-closed';

  bool expanded = false;
  UListElement uElement;

  FolderView(String name, [bool expanded]) : super(name) {
    this.expanded = expanded;

    container.classes.add('explorer-folder');
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

  void toggleBuildingIndicator() {
    if (icon.classes.contains('glyphicons-refresh')) {
      icon.classes.remove('glyphicons-refresh');
      (expanded) ? icon.classes.add(openFolderClass) : icon.classes.add(closedFolderClass);
    } else {
      (expanded) ? icon.classes.remove(openFolderClass) : icon.classes.remove(closedFolderClass);
      icon.classes.add('glyphicons-refresh');
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