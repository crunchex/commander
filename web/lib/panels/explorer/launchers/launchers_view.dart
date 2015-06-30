part of updroid_explorer_launchers;

class LaunchersView extends ExplorerView {
  /// Returns an initialized [PanelView] as a [Future] given all normal constructors.
  ///
  /// Use this instead of calling the constructor directly.
  static Future<LaunchersView> createLaunchersView(int id, DivElement content) {
    Completer c = new Completer();
    c.complete(new LaunchersView(id, content));
    return c.future;
  }

  ParagraphElement placeholderText;
  UListElement uList;

  LaunchersView(int id, DivElement content) :
  super(id, content) {
    this.content = content;

    placeholderText = new ParagraphElement()
      ..classes.add('explorer-placeholder')
      ..text = 'No Launchers found. Create one!';
    explorersDiv.children.add(placeholderText);

    uList = new UListElement()
      ..classes.add("explorer-ul");
//    explorersDiv.append(uList);

    viewWorkspace.classes.remove('glyphicons-folder-open');
    viewWorkspace.classes.addAll(['glyphicons-folder-closed', 'inactive']);
    viewLaunchers.classes.remove('inactive');
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
  final String openFolderClass = 'glyphicons-expand';
  final String closedFolderClass = 'glyphicons-collapse';

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

class LauncherView extends RosEntityView {
  final String fileClass = 'glyphicons-circle-arrow-right';

  bool expanded = false;
  UListElement uElement;

  LauncherView(String name, List<List<String>> args, [bool expanded]) : super(name) {
    this.expanded = expanded;

    container.classes.add('explorer-node');
    icon.classes.add(fileClass);

    uElement = new UListElement()
      ..hidden = true
      ..classes.add('explorer-ul');
    element.children.add(uElement);

    args.forEach((List<String> argument) {
      LIElement li = new LIElement()
        ..classes.add('explorer-li');
      uElement.children.add(li);

      DivElement container = new DivElement()
        ..classes.addAll(['explorer-ros-container', 'explorer-arg-container'])
        ..style.userSelect = 'none';
      li.children.add(container);

      DivElement arg = new DivElement()
        ..classes.add('explorer-arg-name')
        ..text = argument[0];
      container.children.add(arg);

      InputElement argValue = new InputElement()
        ..classes.add('explorer-arg-input');
      if (argument[1] != null) argValue.placeholder = argument[1];
      container.children.add(argValue);
    });
  }

  void toggleExpansion() {
    if (expanded) {
      uElement.hidden = true;
      expanded = false;
    } else {
      uElement.hidden = false;
      expanded = true;
    }
  }

  void expand() {
    uElement.hidden = false;
    expanded = true;
  }

  void collapse() {
    uElement.hidden = true;
    expanded = false;
  }
}