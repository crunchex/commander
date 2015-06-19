library updroid_explorer;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'package:dnd/dnd.dart';
import 'package:path/path.dart' as pathLib;

import '../../context_menu.dart';
import '../../mailbox.dart';
import '../../modal/modal.dart';
import '../panel_controller.dart';

part 'explorer_helper.dart';
part 'explorer_view.dart';
part 'workspaces_view.dart';
part 'nodes_view.dart';

/// [UpDroidConsole] is a client-side class that combines a [Terminal]
/// and [WebSocket] into an UpDroid Commander tab.
class UpDroidExplorer extends PanelController {
  static const String className = 'UpDroidExplorer';

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Add Workspace'},
        {'type': 'toggle', 'title': 'Delete Workspace'},
        {'type': 'toggle', 'title': 'Close Panel'}]},
      {'title': 'Actions', 'items': [
        {'type': 'toggle', 'title': 'Build Workspace'},
        {'type': 'toggle', 'title': 'Clean Workspace'},
        {'type': 'toggle', 'title': 'Upload with Git'},
        {'type': 'toggle', 'title': 'Run Node'}]},
      {'title': 'View', 'items': [
        {'type': 'toggle', 'title': 'Workspaces'},
        {'type': 'toggle', 'title': 'Nodes'}]}
    ];
    return menu;
  }

  // Make dynamic
  bool closed;

  String workspacePath;
  DivElement currentSelected;
  LIElement currentSelectedNode;
  String currentSelectedPath;
  InputElement nodeArgs;

  AnchorElement _dropdown;
  AnchorElement _addWorkspaceButton;
  AnchorElement _deleteWorkspaceButton;
  AnchorElement _cleanButton;
  AnchorElement _buildButton;
  AnchorElement _uploadButton;
  AnchorElement _runButton;
  AnchorElement _workspacesButton;
  AnchorElement _nodesButton;

  Dropzone dzRecycle;
  StreamSubscription outsideClickListener;
  StreamSubscription controlLeave;

  WorkspacesView _workspacesView;
  NodesView _nodesView;

  Map<String, FileSystemEntity> entities = {};
  Map runParams = {};

  List recycleListeners = [];

  StreamController<CommanderMessage> cs;

  UpDroidExplorer(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Explorer', getMenuConfig(), cs, true) {

  }

  Future setUpController() async {
    _addWorkspaceButton = view.refMap['add-workspace'];
    _deleteWorkspaceButton = view.refMap['delete-workspace'];
    _cleanButton = view.refMap['clean-workspace'];
    _buildButton = view.refMap['build-workspace'];
    _uploadButton = view.refMap['upload-with-git'];
    _runButton = view.refMap['run-node'];
    _workspacesButton = view.refMap['workspaces'];
    _nodesButton = view.refMap['nodes'];

    return await WorkspacesView.createWorkspacesView(id, view.content).then((explorerView) {
      _workspacesView = explorerView;

      dzRecycle = new Dropzone(_workspacesView.recycle);
    });
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {
    mailbox.registerCommanderEvent('REQUEST_PARENT_PATH', _requestParentPath);
    mailbox.registerCommanderEvent('CATKIN_NODE_LIST', _catkinNodeList);

    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'SEND_DIRECTORY_PATH', _getDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_PATH', _explorerDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'INITIAL_DIRECTORY_LIST', initialDirectoryList);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_LIST', generateDirectoryList);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_REFRESH', refreshPage);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_ADD', addUpdate);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_REMOVE', removeUpdate);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'CATKIN_NODE_LIST', populateNodes);
  }

  /// Sets up the event handlers for the console.
  void registerEventHandlers() {
    _addWorkspaceButton.onClick.listen((e) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'ADD_WORKSPACE')));
    _deleteWorkspaceButton.onClick.listen((e) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'DELETE_WORKSPACE')));
    _cleanButton.onClick.listen((e) => mailbox.ws.send('[[EXPLORER_WORKSPACE_CLEAN]]'));
    _buildButton.onClick.listen((e) => mailbox.ws.send('[[EXPLORER_WORKSPACE_BUILD]]'));
    _uploadButton.onClick.listen((e) => new UpDroidGitPassModal(cs));
    _runButton.onClick.listen((e) => _runNode());

    _workspacesButton.onClick.listen((e) {
      for(var explorer in _workspacesView.explorersDiv.children) {
        if(explorer.id != 'recycle' && !explorer.classes.contains('control-buttons')) {
          if(!explorer.classes.contains('hidden') && int.parse(explorer.dataset['num']) != num) {
            explorer.classes.add('hidden');
          }
          if(int.parse(explorer.dataset['num']) == num) {
            explorer.classes.remove('hidden');
          }
        }
      }
    });

//    _nodesButton.onClick.listen((e) => showNodes());

    // TODO: cancel when inactive

    var recycleDrag = dzRecycle.onDragEnter.listen((e) => _workspacesView.recycle.classes.add('recycle-entered'));
    var recycleLeave = dzRecycle.onDragLeave.listen((e) => _workspacesView.recycle.classes.remove('recycle-entered'));

    var recycleDrop = dzRecycle.onDrop.listen((e) {
      if (!_workspacesView._explorer.classes.contains('hidden')) {
        var path = getPath(e.draggableElement);

        // Draggable is an empty folder
        if (e.draggableElement.dataset['isDir'] == 'true') {
          LIElement selectedFolder = pathToFile[path];
          selectedFolder.remove();
          removeFileData(selectedFolder, path);
          if (checkContents(selectedFolder) == true) {
            removeSubFolders(selectedFolder);
          }
        }

        mailbox.ws.send('[[EXPLORER_DELETE]]' + path);
      }
    });
    recycleListeners.addAll([recycleDrag, recycleLeave, recycleDrop]);
  }

  void _requestParentPath(CommanderMessage m) {
    if (!_workspacesView._explorer.classes.contains('hidden')) {
      cs.add(new CommanderMessage('UPDROIDEDITOR', 'PARENT_PATH', body: currentSelectedPath));
    }
  }

  void _catkinNodeList(CommanderMessage m) => mailbox.ws.send('[[CATKIN_NODE_LIST]]');

  void _runNode() {
    String runCommand;
    if (nodeArgs.value.isEmpty) {
      runCommand = JSON.encode([runParams['package'], runParams['package-path'], runParams['name']]);
    } else {
      runCommand = JSON.encode([runParams['package'], runParams['package-path'], runParams['name'], nodeArgs.value]);
    }
    mailbox.ws.send('[[CATKIN_RUN]]' + runCommand);
  }

  //\/\/ UpDroidMessage Handlers /\/\//

  void _getDirPath(UpDroidMessage um) => mailbox.ws.send('[[EXPLORER_DIRECTORY_PATH]]');

  void _explorerDirPath(UpDroidMessage um) {
    workspacePath = um.body;
    mailbox.ws.send('[[INITIAL_DIRECTORY_LIST]]');
  }

  //\/\/ Handler Helpers /\/\//

  /// Returns a list of file objects from the flattened string returned from
  /// the server.
  List<FileSystemEntity> fileList(String data) {
    print(data);
    var files = [];

    // Strip the brackets/single-quotes and split by ','.
    data = data.replaceAll(new RegExp(r"(\[|\]|')"), '');
    List<String> entities = data.split(',');

    // Build SimpleFile list our of raw strings.
    for (String entity in entities) {
      if(entity != "") files.add(new FileSystemEntity(entity, workspacePath, ws));
    }
    return files;
  }

  void removeSubFolders(LIElement li) {
    List subFolders = [];
    var children;
    if (li.hasChildNodes()) {
      var ul = li.childNodes;
      children = ul[ul.length - 1].childNodes;
    }
    if (children != null) {
      for (var item in children) {
        if (item.dataset['isDir'] == 'true') {
          subFolders.add(item);
          removeSubFolders(item);
        }
      }
    }
    for (var folder in subFolders) {
      removeFileData(folder, folder.dataset['path']);
    }
  }

  /// Returns a generated [LIElement] with inner HTML based on
  /// the [SimpleFile]'s contents.
  LIElement generateLiHtml(file, [expanded]) {
    LIElement li = new LIElement()
      ..draggable = true
      ..classes.add('explorer-li');

    // Create a span element for the glyphicon
    SpanElement glyphicon = new SpanElement();
    SpanElement glyph = new SpanElement();
    glyph.classes.addAll(['glyphicons', 'glyphicons-folder-closed', "list-folder"]);
    var glyphType = (file.isDirectory) ? 'glyphicons-folder-open' : 'glyphicons-file';
    glyphicon.classes.addAll(['glyphicons', glyphType]);
//    dropSetup(glyphicon, file);
//    dropSetup(glyph, file);

    DivElement fileContainer = new DivElement();
    setupHighlighter(fileContainer);
    fileContainer.classes.add('file-container');
    li.children.add(fileContainer);
    fileContainer.children.add(glyphicon);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement()
      ..classes.add('filename')
      ..text = file.name;
    fileContainer.children.add(filename);

    if (file.isDirectory) {
      li.dataset['expanded'] = 'false';
      UListElement ul = new UListElement();
      ul..classes.addAll(['explorer', 'explorer-ul']);
      li.children.add(ul);

      ul.classes.add("hidden");
      glyphicon.replaceWith(glyph);
      li.dataset['expanded'] = 'false';

      if (expanded == true) {
        glyph.replaceWith(glyphicon);
        li.dataset['expanded'] = 'true';
        ul.classes.remove("hidden");
      }

      glyphicon.onClick.listen((e) {
        ul.classes.add("hidden");
        glyphicon.replaceWith(glyph);
        li.dataset['expanded'] = 'false';
      });

      glyph.onClick.listen((e) {
        glyph.replaceWith(glyphicon);
        li.dataset['expanded'] = 'true';
        ul.classes.remove("hidden");
      });

      glyph.onContextMenu.listen((e) {
        e.preventDefault();

        List menu = [
          {'type': 'toggle', 'title': 'New File', 'handler': () => mailbox.ws.send('[[EXPLORER_NEW_FILE]]' + file.path)},
          {'type': 'toggle', 'title': 'New Folder', 'handler': () => mailbox.ws.send('[[EXPLORER_NEW_FOLDER]]' + file.path + '/untitled')},
          {'type': 'toggle', 'title': 'Delete', 'handler': () => mailbox.ws.send('[[EXPLORER_DELETE]]' + file.path)}];

        new ContextMenu(glyph, menu);

      });
    }

    return li;
  }

  /// Sets up a [Dropzone] for the [SpanElement] to handle file moves.
  void dropSetup(SpanElement span, FileSystemEntity file) {
    if (file.isDirectory) {
      Dropzone d = new Dropzone(span);

      d.onDragEnter.listen((e) => span.classes.add('span-entered'));
      d.onDragLeave.listen((e) => span.classes.remove('span-entered'));

      d.onDrop.listen((e) {
        if (e.draggableElement.className.contains('explorer-li')) {
          // The draggable is an existing file/folder.
          var currentPath = getPath(e.draggableElement);
          var newPath = '${getPath(span.parent.parent)}/${getName(e.draggableElement)}';
          bool duplicate;
          LIElement item = pathToFile[currentPath];

          // Check for duplicate file name
          pathToFile.containsKey(newPath) ? duplicate = true : duplicate = false;
          bool alert = false;
          if (duplicate == true) {
            alert = true;
          }

          // The draggable is an empty folder

          if (e.draggableElement.dataset['isDir'] == 'true') {
            bool send = true;
            if (getPath(span.parent.parent).contains(getPath(e.draggableElement))) {
              send = checkNested(getPath(span.parent.parent), getPath(e.draggableElement));
            }
            // Avoid an exception thrown when the new name already exists or dragging to same folder.

            if (currentPath != newPath && duplicate == false && send == true) {
              if (item.lastChild.hasChildNodes() == false) {
                mailbox.ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                var name = getName(e.draggableElement);
                removeFileData(e.draggableElement, currentPath);
                newElementFromFile(
                    new FileSystemEntity('D:' + getPath(span.parent.parent) + '/' + name, workspacePath, ws));
                item.remove();
              } else if (checkContents(item) == true) {
                mailbox.ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                cs.add(new CommanderMessage('UPDROIDEDITOR', 'FILE_UPDATE', body: [currentPath, newPath]));
                removeSubFolders(item);
                removeFileData(e.draggableElement, currentPath);
                mailbox.ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
                item.remove();
              } else {
                mailbox.ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                cs.add(new CommanderMessage('UPDROIDEDITOR', 'FILE_UPDATE', body: [currentPath, newPath]));
                item.remove();
                removeFileData(e.draggableElement, currentPath);
              }
            }
          } else {
            if (currentPath != newPath &&
            duplicate == false &&
            !getPath(span.parent.parent).contains(getPath(e.draggableElement))) {
              mailbox.ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              cs.add(new CommanderMessage('UPDROIDEDITOR', 'FILE_UPDATE', body: [currentPath, newPath]));
            }
          }

          if (alert == true && item != duplicate) {
            window.alert("Cannot move here, file name already exists");
          }
        } else if (e.draggableElement.classes.contains('file')) {
          mailbox.ws.send('[[EXPLORER_NEW_FILE]]' + getPath(span.parent.parent));
        } else {
          mailbox.ws.send('[[EXPLORER_NEW_FOLDER]]' + getPath(span.parent.parent) + '/untitled');
        }
      });
    }
  }

  void setupHighlighter(DivElement div) {
    div.onClick.listen((e) {
      // This case only covers the first click
      // Stores the necessary data for all future clicks
      if (currentSelected != null) {
        currentSelected.classes.remove('highlighted');
      }
      div.classes.add('highlighted');
      currentSelected = div;
      div.parent.dataset['isDir'] == 'true'
      ? currentSelectedPath = getPath(div.parent)
      : currentSelectedPath = pathLib.dirname(getPath(div.parent));
    });
  }

  void populateNodes(UpDroidMessage um) {
    List<Map> nodeList = JSON.decode(um.body);
    Map packageMap = _nodesView.createPackageList(nodeList);

    for (var packageNode in nodeList) {
      if(!packageNode['node'].contains('.xml')) {
        var element = _nodesView.createNodeLi(cs, packageNode);
        var listToAppend = packageMap[packageNode['package']];
        listToAppend.append(element);
        setupNodeHighlighter(element);
      }
    }
  }

  void setupNodeHighlighter (LIElement li) {
    li.onClick.listen((e) {
      if(currentSelectedNode != null) {
        currentSelectedNode.classes.remove('highlighted');
        nodeArgs.classes.add('hidden');
      }
      li.classes.add('highlighted');
      currentSelectedNode = li;
      nodeArgs = li.lastChild;
      runParams.addAll({'name' : li.dataset['name'], 'package' : li.dataset['package'], 'package-path' : li.dataset['package-path']});
      nodeArgs.classes.remove('hidden');
    });
  }

  /// Handles file renaming with a double-click event.
  void renameEventHandler(LIElement li, FileSystemEntity file) {
    bool refresh = false;
    bool renameFinish = false;
    bool folder = false;

    if (li.dataset['isDir'] == 'true') {
      folder = true;
      if (checkContents(li) == true) {
        refresh = true;
      }
    }

    if (!li.className.contains('editing')) {
      li.classes.add('editing');

      InputElement input = new InputElement();
      input.value = '${li.dataset['name']}';

      Element outside = querySelector('.container-fluid');

      outsideClickListener = outside.onClick.listen((e) {
        if (e.target != input && renameFinish == false) {
          mailbox.ws.send('[[EXPLORER_DIRECTORY_LIST]]');
          outsideClickListener.cancel();
        }
      });

      input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          renameFinish = true;
          var newPath = pathLib.join(pathLib.dirname(file.path), input.value);

          bool duplicate = pathToFile.containsKey(newPath);
          if (duplicate == false) {
            mailbox.ws.send('[[EXPLORER_RENAME]]' + file.path + ':divider:' + newPath);
            cs.add(new CommanderMessage('UPDROIDEDITOR', 'FILE_UPDATE', body: [file.path, newPath]));
            if (folder == true) {
              removeFileData(li, file.path);
            }
          }

          // TODO: Create a overwrite option in case of existing file name

          if (duplicate == true) {
            if (duplicate == li) {
              mailbox.ws.send('[[EXPLORER_DIRECTORY_LIST]]');
            } else {
              window.alert("File name already exists");
              mailbox.ws.send('[[EXPLORER_DIRECTORY_LIST]]');
            }
          }

          // Remove this element once editing is complete, as the new one will soon appear.
          if (file.path != newPath) {
            UListElement ul = li.parent;
            ul.children.remove(li);
          }

          // Put the element back in the case that rename is canceled

          if (file.path == newPath) {
            li.remove();
            if (file.isDirectory == true) {
              mailbox.ws.send('[[EXPLORER_DIRECTORY_LIST]]');
            }
          }

          if (refresh == true) {
            removeSubFolders(li);
            mailbox.ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
          }

          // Create a folder icon if the item renamed was an empty folder

          if (file.isDirectory == true && li.lastChild.hasChildNodes() == false) {
            newElementFromFile(new FileSystemEntity('D:$newPath', workspacePath, mailbox.ws));
          }
        } else {
          // TODO: need to make field width scale to the user's input.
          // Using a 'contenteditable' <span> instead of an <input> is a possible option.
          //input.size = auto;
        }
      });

      // Replace child 1 (span filename) with input.
      li.children[0].children[1] = input;
      input.focus();
      input.select();
    }
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

    FileSystemEntity entity = new FileSystemEntity(data, workspacePath, mailbox.ws);
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
      entities[path].cleanup();
      entities.remove(path);
      return;
    }

    // More work for a directory where we recursively delete (sort of).
    List<String> keysWithPath = entities.keys.where((String key) => key.contains(path));
    List<String> entityKeys = new List.from(keysWithPath);
    entityKeys.forEach((String key) {
      entities[key].cleanup();
      entities.remove(key);
    });
  }

  void destroyRecycleListeners() {
    for (var listener in recycleListeners) {
      listener.cancel();
    }
  }

  /// Sets up a new HTML element from a SimpleFile.
  Future newElementFromFile(FileSystemEntity file, [bool expanded]) {
    Completer completer = new Completer();

    LIElement li = generateLiHtml(file, expanded);

    // Register double-click event handler for file renaming.
    li.children[0].children[1].onDoubleClick.listen((e) {
      renameEventHandler(li, file);
      // To prevent the rename event from propagating up the directory tree.
      e.stopPropagation();
    });

    // Set up drag and drop for file open & delete.
    dragSetup(li, file);

    UListElement dirElement;
    if (file.parent == '' && !file.path.contains('/.')) {
      dirElement = _workspacesView.uList;
      dirElement.append(li);
    } else if (!file.path.contains('/.')) {
      dirElement = ulInfo[pathLib.dirname(file.path)];
      dirElement.append(li);
    }

    completer.complete(true);
    return completer.future;
  }

  /// First Directory List Generation
  void initialDirectoryList(UpDroidMessage um) {
    List<String> fileStrings = JSON.decode(um.body);

    _workspacesView.uList.innerHtml = '';

    for (String rawString in fileStrings) {
      addFileSystemEntity(rawString);
    }
  }

  /// Redraws all file explorer views.
  void generateDirectoryList(UpDroidMessage um) {
    String raw = um.body;
    var files = fileList(raw);
    var folderStateList = {};
    for (var file in files) {
      if (file.isDirectory == true) {
        LIElement folder = pathToFile[file.path];
        if (folder != null) {
          folder.dataset['expanded'] == "true" ? folderStateList[file] = true : folderStateList[file] = false;
        }
      }
    }

    ulInfo.clear();
    fileInfo.clear();
    pathToFile.clear();

    // Set the explorer list to empty for a full refresh.
    UListElement explorer = _workspacesView.uList;
    explorer.innerHtml = '';

    for (FileSystemEntity file in files) {
      file.isDirectory == true ? newElementFromFile(file, folderStateList[file]) : newElementFromFile(file);
    }
  }

  /// Update the explorer view in case of nested folder movement to cover for empty folders
  void refreshPage(UpDroidMessage um) {
    String raw = um.body;
    var files = fileList(raw);

    for (FileSystemEntity file in files) {
      //only check for folders
      if (file.isDirectory == true) {
        LIElement curFolder = pathToFile[file.path];
        if (curFolder == null) {
          newElementFromFile(file);
        }
      }
    }
  }

  void cleanUp() {

  }
}

/// Container class that extracts data from the raw file text passed in from
/// the server over [WebSocket]. Primarily used for generating the HTML views
/// in the file explorer that represent the filesystem.
class FileSystemEntity {
  String path, workspacePath, name, parent;
  bool isDirectory;
  WebSocket ws;
  FileSystemEntityView view;

  FileSystemEntity(String raw, String workspacePath, this.ws) {
    List<String> rawList = raw.split(':');
    isDirectory = rawList[0] == 'D' ? true : false;
    path = pathLib.normalize(rawList[1]);
    this.workspacePath = pathLib.normalize(workspacePath);

    name = getNameFromPath(path, workspacePath);
    parent = getParentFromPath(path, workspacePath);

    isDirectory ? setUpFolderView() : setUpFileView();

    print('workspacePath: $workspacePath, path: $path, name: $name, parent: $parent');
  }

  void setUpFolderView() {
    view = new FolderView(name);
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
    view.container.onContextMenu.listen((e) {
      e.preventDefault();
      List menu = [
        {'type': 'toggle', 'title': 'Rename', 'handler': rename},
        {'type': 'toggle', 'title': 'Delete', 'handler': () => ws.send('[[EXPLORER_DELETE]]' + path)}];
      ContextMenu.createContextMenu(e.page, menu);
    });
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

  void cleanup() {
    //_contextListeners.forEach((StreamSubscription listener) => listener.cancel());
    view.cleanup();
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