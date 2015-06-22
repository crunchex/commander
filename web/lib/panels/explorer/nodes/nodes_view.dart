part of updroid_explorer_nodes;

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

    uList = new UListElement()
      ..classes.add("explorer-ul");
    explorersDiv.append(uList);

    DivElement toolbar = new DivElement()
      ..classes.add('toolbar');
    explorersDiv.children.add(toolbar);
  }

  void cleanUp() {
    content.innerHtml = '';
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

  UListElement uElement;

  NodeView(String name, List<List<String>> args) : super(name) {
    container.classes.add('explorer-node');
    icon.classes.add(fileClass);

    uElement = new UListElement()
      ..classes.add('explorer-ul');
    element.children.add(uElement);

    args.forEach((List<String> argument) {
      LIElement li = new LIElement()
        ..classes.add('explorer-li');
      uElement.children.add(li);

      DivElement container = new DivElement()
        ..classes.add('explorer-ros-container')
        ..style.userSelect = 'none';
      li.children.add(container);

      SpanElement arg = new SpanElement()
        ..text = argument[0];
      container.children.add(arg);


      InputElement argValue = new InputElement();
      if (argument[1] != null) argValue.placeholder = argument[1];
      container.children.add(argValue);
    });
  }
}