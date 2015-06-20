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
part 'workspaces_controller.dart';
part 'nodes_controller.dart';
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
        {'type': 'divider', 'title': 'Workspaces'},
        {'type': 'toggle', 'title': 'Build Workspace'},
        {'type': 'toggle', 'title': 'Clean Workspace'},
        {'type': 'toggle', 'title': 'Upload with Git'},
        {'type': 'divider', 'title': 'Nodes'},
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

  StreamSubscription outsideClickListener;
  StreamSubscription controlLeave;

  Map<String, FileSystemEntity> entities = {};
  Map runParams = {};

  List recycleListeners = [];

  ExplorerController controller;
  StreamController<CommanderMessage> cs;

  UpDroidExplorer(int id, int col, StreamController<CommanderMessage> cs) :
  super(id, col, className, 'Explorer', getMenuConfig(), cs, true) {

  }

  Future setUpController() {
    _addWorkspaceButton = view.refMap['add-workspace'];
    _deleteWorkspaceButton = view.refMap['delete-workspace'];
    _cleanButton = view.refMap['clean-workspace'];
    _buildButton = view.refMap['build-workspace'];
    _uploadButton = view.refMap['upload-with-git'];
    _runButton = view.refMap['run-node'];
    _workspacesButton = view.refMap['workspaces'];
    _nodesButton = view.refMap['nodes'];

    controller = new UpDroidWorkspaces(id, view.content, mailbox);
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

  void cleanUp() {

  }
}

abstract class ExplorerController {

}