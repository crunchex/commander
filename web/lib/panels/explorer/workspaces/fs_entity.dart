part of updroid_explorer_workspaces;

/// Container class that extracts data from the raw file text passed in from
/// the server over [WebSocket]. Primarily used for generating the HTML views
/// in the file explorer that represent the filesystem.
class FileSystemEntity {
  String path, workspacePath, name, parent;
  bool isDirectory, isPackage, selected;
  WebSocket ws;
  FileSystemEntityView view;

  bool _selectEnabled;

  FileSystemEntity(String raw, String workspacePath, this.ws) {
    selected = false;
    _selectEnabled = true;

    List<String> rawList = raw.split(':');
    isDirectory = rawList[0] == 'D' ? true : false;
    path = pathLib.normalize(rawList[1]);
    this.workspacePath = pathLib.normalize(workspacePath);

    name = getNameFromPath(path, workspacePath);
    parent = getParentFromPath(path, workspacePath);

    if (isDirectory) {
      isPackage = false;
      setUpFolderView();
    } else {
      setUpFileView();
    }

    //print('workspacePath: $workspacePath, path: $path, name: $name, parent: $parent');
  }

  void setUpFolderView() {
    view = new FolderView(name);

    view.container.onClick.listen((e) {
      if (_selectEnabled) {
        toggleSelected();
        _selectEnabled = false;

        new Timer(new Duration(milliseconds: 500), () {
          _selectEnabled = true;
        });
      }
    });

    view.container.onDoubleClick.listen((e) {
      FolderView folderView = view;
      folderView.toggleExpansion();
      select();
    });

    view.container.onContextMenu.listen((e) {
      e.preventDefault();
      List menu = [
        {'type': 'toggle', 'title': 'New File', 'handler': () => ws.send('[[NEW_FILE]]' + path)},
        {'type': 'toggle', 'title': 'New Folder', 'handler': () => ws.send('[[NEW_FOLDER]]' + path + '/untitled')},
        {'type': 'toggle', 'title': 'Rename', 'handler': rename},
        {'type': 'toggle', 'title': 'Delete', 'handler': () => ws.send('[[DELETE]]' + path)}];

      if (isPackage) {
        menu.add({'type': 'divider', 'title': ''});
        menu.add({'type': 'toggle', 'title': 'Build', 'handler': build});
      }
      ContextMenu.createContextMenu(e.page, menu);
    });
  }

  void setUpFileView() {
    view = new FileView(name);

    view.container.onClick.listen((e) {
      if (_selectEnabled) {
        toggleSelected();
        _selectEnabled = false;

        new Timer(new Duration(milliseconds: 500), () {
          _selectEnabled = true;
        });
      }
    });

    view.container.onContextMenu.listen((e) {
      e.preventDefault();
      List menu = [
        {'type': 'toggle', 'title': 'Rename', 'handler': rename},
        {'type': 'toggle', 'title': 'Delete', 'handler': () => ws.send('[[DELETE]]' + path)}];
      ContextMenu.createContextMenu(e.page, menu);
    });
  }

  void toggleSelected() => selected ? deselect() : select();

  void select() {
    view.select();
    selected = true;
  }

  void deselect() {
    view.deselect();
    selected = false;
  }

  void build() {
    print('$name $path');
    // Special case if workspace folder.
    if (name != path.split('/').last) {
      ws.send('[[WORKSPACE_BUILD]]');
      return;
    }

    ws.send('[[BUILD_PACKAGE]]' + name);
  }

  void rename() {
    InputElement renameInput = view.startRename();
    renameInput.onKeyUp.listen((e) {
      if (e.keyCode == KeyCode.ENTER) {
        String newPath = pathLib.normalize('$parent/${renameInput.value}');
        ws.send('[[RENAME]]$path:$newPath');
        view.completeRename(renameInput);
      }
    });

    document.body.onClick.first.then((_) => view.completeRename(renameInput));
  }

  void cleanUp() {
    //_contextListeners.forEach((StreamSubscription listener) => listener.cancel());
    view.cleanUp();
  }

  static String getNameFromPath(String path, String workspacePath) {
    List<String> pathList = path.split('/');
    return (path == '$workspacePath/src') ? pathList[pathList.length - 2] : pathList.last;
  }

  static String getParentFromPath(String path, String workspacePath) {
    List<String> pathList = path.split('/');
    if (path == '$workspacePath/src') {
      return null;
    }

    String parent = '';
    pathList.sublist(0, pathList.length - 1).forEach((String s) => parent += '$s/');
    return pathLib.normalize(parent);
  }
}