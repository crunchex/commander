part of updroid_explorer;

class UpDroidWorkspaces implements ExplorerController {
  PanelView _view;
  WorkspacesView _workspacesView;
  Mailbox _mailbox;

  AnchorElement _cleanButton;
  AnchorElement _buildButton;
  AnchorElement _uploadButton;

  Dropzone dzRecycle;

  Map<String, FileSystemEntity> entities = {};
  String workspacePath;

  UpDroidWorkspaces(int id, this.workspacePath, PanelView view, Mailbox mailbox) {
    _view = view;
    _mailbox = mailbox;

    registerMailbox();

    WorkspacesView.createWorkspacesView(id, _view.content).then((workspacesView) {
      _workspacesView = workspacesView;

      _cleanButton = _view.refMap['clean-workspace'];
      _buildButton = _view.refMap['build-workspace'];
      _uploadButton = _view.refMap['upload-with-git'];

      dzRecycle = new Dropzone(_workspacesView.recycle);

      _mailbox.ws.send('[[INITIAL_DIRECTORY_LIST]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'INITIAL_DIRECTORY_LIST', initialDirectoryList);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_ADD', addUpdate);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_REMOVE', removeUpdate);
  }

  void registerEventHandlers() {
    _cleanButton.onClick.listen((e) => _mailbox.ws.send('[[EXPLORER_WORKSPACE_CLEAN]]'));
    _buildButton.onClick.listen((e) => _mailbox.ws.send('[[EXPLORER_WORKSPACE_BUILD]]'));
//    _uploadButton.onClick.listen((e) => new UpDroidGitPassModal(cs));
  }

  /// Sets up a [Draggable] for the existing [LIElement] to handle file open and delete.
  void dragSetup(LIElement li, FileSystemEntity file) {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d = new Draggable(li, avatarHandler: new AvatarHandler.clone());

    // Dragging through nested dropzones appears to be glitchy.
    d.onDragStart.listen((event) {
      d.avatarHandler.avatar.children.first.classes.remove('highlighted');
      if (pathLib.dirname(li.dataset['path']) != workspacePath) _workspacesView.drop.classes.add('file-drop-ondrag');
      _workspacesView.recycle.classes.add('recycle-ondrag');
      ElementList<SpanElement> spanList = querySelectorAll('.glyphicons-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.add('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_ADD', body: 'updroideditor-ondrag'));
      }
    });

    d.onDragEnd.listen((event) {
      _workspacesView.drop.classes.remove('file-drop-ondrag');
      _workspacesView.recycle.classes.remove('recycle-ondrag');
      ElementList<SpanElement> spanList = querySelectorAll('.glyphicons-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.remove('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_REMOVE', body: 'updroideditor-ondrag'));
      }
    });
  }

  /// Handles an Explorer add update for a single file.
  void addUpdate(UpDroidMessage um) => addFileSystemEntity(um.body);

  void addFileSystemEntity(String data) {
    String path = data.split(':')[1];

    // Don't do anything if the entity is already in the system.
    if (entities.containsKey(path)) return;

    // Recursively add a parent that isn't in the system yet.
    String parent = FileSystemEntity.getParentFromPath(path, workspacePath);
    if (parent != null && !entities.containsKey(parent)) addFileSystemEntity('D:$parent');

    FileSystemEntity entity = new FileSystemEntity(data, workspacePath, _mailbox.ws);
    entities[entity.path] = entity;

    // Special case for the workspace src directory (root node).
    if (entity.parent == null) {
      _workspacesView.uList.children.add(entity.view.element);
      return;
    }

    FolderView parentFolder = entities[entity.parent].view;
    parentFolder.uElement.children.add(entity.view.element);
  }

  /// Handles an Explorer remove update for a single file.
  void removeUpdate(UpDroidMessage um) => removeFileSystemEntity(um.body);

  void removeFileSystemEntity(String data) {
    List<String> split = data.split(':');
    String type = split[0];
    String path = split[1];

    // Don't do anything if the entity is not in the system.
    if (!entities.containsKey(path)) return;

    // Simple case for a file.
    if (type == 'F') {
      entities[path].cleanUp();
      entities.remove(path);
      return;
    }

    // More work for a directory where we recursively delete (sort of).
    List<String> keysWithPath = entities.keys.where((String key) => key.contains(path));
    List<String> entityKeys = new List.from(keysWithPath);
    entityKeys.forEach((String key) {
      entities[key].cleanUp();
      entities.remove(key);
    });
  }

  /// First Directory List Generation
  void initialDirectoryList(UpDroidMessage um) {
    List<String> fileStrings = JSON.decode(um.body);

    _workspacesView.uList.innerHtml = '';

    for (String rawString in fileStrings) {
      addFileSystemEntity(rawString);
    }
  }
}

/// Container class that extracts data from the raw file text passed in from
/// the server over [WebSocket]. Primarily used for generating the HTML views
/// in the file explorer that represent the filesystem.
class FileSystemEntity {
  String path, workspacePath, name, parent;
  bool isDirectory, selected;
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

    isDirectory ? setUpFolderView() : setUpFileView();

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
        {'type': 'toggle', 'title': 'New File', 'handler': () => ws.send('[[EXPLORER_NEW_FILE]]' + path)},
        {'type': 'toggle', 'title': 'New Folder', 'handler': () => ws.send('[[EXPLORER_NEW_FOLDER]]' + path + '/untitled')},
        {'type': 'toggle', 'title': 'Rename', 'handler': rename},
        {'type': 'toggle', 'title': 'Delete', 'handler': () => ws.send('[[EXPLORER_DELETE]]' + path)}];
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
        {'type': 'toggle', 'title': 'Delete', 'handler': () => ws.send('[[EXPLORER_DELETE]]' + path)}];
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

  void rename() {
    InputElement renameInput = view.startRename();
    renameInput.onKeyUp.listen((e) {
      if (e.keyCode == KeyCode.ENTER) {
        String newPath = pathLib.normalize('$parent/${renameInput.value}');
        ws.send('[[EXPLORER_RENAME]]$path:$newPath');
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