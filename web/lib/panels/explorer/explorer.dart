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

  Map editors = {};
  Map editorListeners = {};
  Map fileInfo = {};
  Map pathToFile = {};
  Map ulInfo = {};
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
    mailbox.registerCommanderEvent('EDITOR_READY', _editorReady);
    mailbox.registerCommanderEvent('REQUEST_PARENT_PATH', _requestParentPath);
    mailbox.registerCommanderEvent('CATKIN_NODE_LIST', _catkinNodeList);

    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'SEND_DIRECTORY_PATH', _getDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_PATH', _explorerDirPath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'INITIAL_DIRECTORY_LIST', initialDirectoryList);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_LIST', generateDirectoryList);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_REFRESH', refreshPage);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_ADD', addUpdate);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_REMOVE', removeUpdate);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_CLEAN', _relayWorkspaceClean);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_BUILD', _relayWorkspaceBuild);
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

  void _editorReady(CommanderMessage m) {
    var num = m.body[0];
    // Editor num
    var dropDiv = m.body[1];

    var dzEditor = new Dropzone(dropDiv);
    if (editorListeners != null) {
      editorListeners.putIfAbsent(dzEditor, () => createEditorListeners(dzEditor));
      editors.putIfAbsent(dzEditor, () => num);
      cs.add(new CommanderMessage('UPDROIDEDITOR', 'PASS_EDITOR_INFO', body: [num, dzEditor]));
    }
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

  void _relayWorkspaceClean(UpDroidMessage um) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'WORKSPACE_CLEAN'));
  void _relayWorkspaceBuild(UpDroidMessage um) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'WORKSPACE_BUILD', body: um.body));

  //\/\/ Handler Helpers /\/\//

  // TODO: cancel when inactive
  List<StreamSubscription> createEditorListeners(Dropzone dzEditor) {
    var enter = dzEditor.onDragEnter.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_ADD', body: 'updroideditor-entered'));
      }
    });

    var leave = dzEditor.onDragLeave
    .listen((e) => cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_REMOVE', body: 'updroideditor-entered')));

    var drop = dzEditor.onDrop.listen((e) {
      if (!_workspacesView._explorer.classes.contains('hidden')) {
        var isDir = e.draggableElement.dataset['isDir'];
        if (isDir == 'false') {
          var num = editors[dzEditor];
          cs.add(new CommanderMessage('UPDROIDEDITOR', 'OPEN_FILE', body: [num, getPath(e.draggableElement)]));
        }
      }
    });
    return [enter, leave, drop];
  }

  /// Returns a list of file objects from the flattened string returned from
  /// the server.
  List<SimpleFile> fileList(String data) {
    var files = [];

    // Strip the brackets/single-quotes and split by ','.
    data = data.replaceAll(new RegExp(r"(\[|\]|')"), '');
    List<String> entities = data.split(',');

    // Build SimpleFile list our of raw strings.
    for (String entity in entities) {
      if(entity != "") files.add(new SimpleFile.fromDirectoryList(entity, workspacePath));
    }
    return files;
  }

  /// Functions for updating tracked file info
  void removeFileData([LIElement li, String path]) {
    if (li != null) {
      fileInfo.remove(li);
    }
    if (path != null) {
      pathToFile.remove(path);
      ulInfo.remove(path);
    }
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

  String getName(LIElement li) {
    return fileInfo[li][0];
  }

  String getPath(LIElement li) {
    return fileInfo[li][1];
  }

  /// Returns a generated [LIElement] with inner HTML based on
  /// the [SimpleFile]'s contents.
  LIElement generateLiHtml(file, [expanded]) {
    LIElement li = new LIElement()
      ..dataset['name'] = file.name
      ..dataset['path'] = file.path
      ..dataset['exp'] = id.toString()
      ..dataset['isDir'] = file.isDirectory.toString()
      ..draggable = true
      ..classes.add('explorer-li');

    fileInfo.putIfAbsent(li, () => [file.name, file.path]);
    pathToFile.putIfAbsent(file.path, () => li);
    // Create a span element for the glyphicon
    SpanElement glyphicon = new SpanElement();
    SpanElement glyph = new SpanElement();
    glyph.classes.addAll(['glyphicons', 'glyphicons-folder-closed', "list-folder"]);
    var glyphType = (file.isDirectory) ? 'glyphicons-folder-open' : 'glyphicons-file';
    glyphicon.classes.addAll(['glyphicons', glyphType]);
    dropSetup(glyphicon, file);
    dropSetup(glyph, file);
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

      ulInfo.putIfAbsent(file.path, () => ul);

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
  void dropSetup(SpanElement span, SimpleFile file) {
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
                    new SimpleFile.fromPath(getPath(span.parent.parent) + '/' + name, workspacePath, true));
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

  void cancelEditorListeners(List streams) {
    for (var stream in streams) {
      stream.cancel();
    }
  }

  void destroyEditorListeners() {
    editorListeners.forEach((k,v) => cancelEditorListeners(v));
    editorListeners = null;
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
  void renameEventHandler(LIElement li, SimpleFile file) {
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
            newElementFromFile(new SimpleFile.fromPath(newPath, workspacePath, true));
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
  void dragSetup(LIElement li, SimpleFile file) {
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
  void addUpdate(UpDroidMessage um) {
    String path = um.body;
    SimpleFile sFile = new SimpleFile.fromPath(path, workspacePath, false);
    var parentPath = pathLib.dirname(sFile.path);

    // Try to detect the parent, and if it doesn't exist then create the element for it.
    String curPath = '';

    // Iterate through the path checking to see if the folder exists
    var split = parentPath.replaceFirst(pathLib.normalize(workspacePath), '').split('/');
    for (int i = 1; i < split.length; i++) {
      curPath += split[i];
      LIElement curLi = pathToFile['${pathLib.join(workspacePath, curPath)}'];
      if (curLi == null && pathLib.join(workspacePath, curPath) != workspacePath) {
        newElementFromFile(new SimpleFile.fromPath(pathLib.join(workspacePath, curPath), workspacePath, true))
        .then((result) {});
      }
      if (i != split.length - 1) {
        curPath += '/';
      }
    }
    newElementFromFile(sFile);
  }

  /// Handles an Explorer remove update for a single file.
  void removeUpdate(UpDroidMessage um) {
    String path = um.body;
    LIElement li = pathToFile[path];

    // Case to deal with removeUpdate grabbing null objects when items are renamed
    if (li != null) li.remove();

    removeFileData(li, path);
  }

  void destroyRecycleListeners() {
    for (var listener in recycleListeners) {
      listener.cancel();
    }
  }

  /// Sets up a new HTML element from a SimpleFile.
  Future newElementFromFile(SimpleFile file, [bool expanded]) {
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
    if (file.parentDir == '' && !file.path.contains('/.')) {
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
    var files = fileList(um.body);

    _workspacesView.uList.innerHtml = '';

    for (SimpleFile file in files) {
      newElementFromFile(file, false);
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

    for (SimpleFile file in files) {
      file.isDirectory == true ? newElementFromFile(file, folderStateList[file]) : newElementFromFile(file);
    }
  }

  /// Update the explorer view in case of nested folder movement to cover for empty folders
  void refreshPage(UpDroidMessage um) {
    String raw = um.body;
    var files = fileList(raw);

    for (SimpleFile file in files) {
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
class SimpleFile {
  String name;
  String parentDir;
  String path;
  bool isDirectory;

  String workspaceName;

  SimpleFile.fromDirectoryList(String raw, String prefix) {
    workspaceName = prefix.split('/').last;
    String workingString = stripFormatting(raw, prefix);
    getData(workingString);
  }

  SimpleFile.fromPath(String raw, String prefix, bool isDir) {
    workspaceName = prefix.split('/').last;
    path = raw.replaceAll(r'\', '');
    isDirectory = isDir;
    raw = raw.replaceFirst(prefix, '');
    getData(raw);
  }

  String stripFormatting(String raw, String prefix) {
    raw = raw.trim();
    isDirectory = raw.startsWith('Directory: ') ? true : false;
    raw = raw.replaceFirst(new RegExp(r'(Directory: |File: )'), '');

    path = raw;
    raw = raw.replaceFirst(prefix, '');
    return raw;
  }

  void getData(String fullPath) {
    List<String> pathList = fullPath.split('/');
    // Change the name of the top-level src folder to the name
    // of the workspace (but leave path unchanged.
    name = (pathList.last == 'src') ? workspaceName : pathList[pathList.length - 1];

    if (pathList.length > 1) {
      parentDir = pathList[pathList.length - 2];
    } else {
      parentDir = '';
    }
  }
}