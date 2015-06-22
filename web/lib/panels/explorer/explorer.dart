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

  DivElement currentSelected;
  LIElement currentSelectedNode;
  String currentSelectedPath;
  InputElement nodeArgs;

  AnchorElement _dropdown;
  AnchorElement _addWorkspaceButton;
  AnchorElement _deleteWorkspaceButton;
  AnchorElement _workspacesButton;
  AnchorElement _nodesButton;

  StreamSubscription outsideClickListener;
  StreamSubscription controlLeave;
  String workspacePath;

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
    _workspacesButton = view.refMap['workspaces'];
    _nodesButton = view.refMap['nodes'];
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_OPEN, 'REQUEST_WORKSPACE_PATH', _requestWorkspacePath);
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'EXPLORER_DIRECTORY_PATH', _explorerDirPath);
  }

  /// Sets up the event handlers for the console.
  void registerEventHandlers() {
    _addWorkspaceButton.onClick.listen((e) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'ADD_WORKSPACE')));
    _deleteWorkspaceButton.onClick.listen((e) => cs.add(new CommanderMessage('UPDROIDCLIENT', 'DELETE_WORKSPACE')));
    _workspacesButton.onClick.listen((e) => _showWorkspacesController());
    _nodesButton.onClick.listen((e) => _showNodesController());
//    _runButton.onClick.listen((e) => _runNode());
//
//    _workspacesButton.onClick.listen((e) {
//      for(var explorer in _workspacesView.explorersDiv.children) {
//        if(explorer.id != 'recycle' && !explorer.classes.contains('control-buttons')) {
//          if(!explorer.classes.contains('hidden') && int.parse(explorer.dataset['num']) != num) {
//            explorer.classes.add('hidden');
//          }
//          if(int.parse(explorer.dataset['num']) == num) {
//            explorer.classes.remove('hidden');
//          }
//        }
//      }
//    });

    // TODO: cancel when inactive

//    var recycleDrag = dzRecycle.onDragEnter.listen((e) => _workspacesView.recycle.classes.add('recycle-entered'));
//    var recycleLeave = dzRecycle.onDragLeave.listen((e) => _workspacesView.recycle.classes.remove('recycle-entered'));
//
//    var recycleDrop = dzRecycle.onDrop.listen((e) {
//      if (!_workspacesView._explorer.classes.contains('hidden')) {
//        var path = getPath(e.draggableElement);
//
//        // Draggable is an empty folder
//        if (e.draggableElement.dataset['isDir'] == 'true') {
//          LIElement selectedFolder = pathToFile[path];
//          selectedFolder.remove();
//          removeFileData(selectedFolder, path);
//          if (checkContents(selectedFolder) == true) {
//            removeSubFolders(selectedFolder);
//          }
//        }
//
//        mailbox.ws.send('[[EXPLORER_DELETE]]' + path);
//      }
//    });
//    recycleListeners.addAll([recycleDrag, recycleLeave, recycleDrop]);
  }

  //\/\/ UpDroidMessage Handlers /\/\//

  void _requestWorkspacePath(UpDroidMessage um) => mailbox.ws.send('[[REQUEST_WORKSPACE_PATH]]');

  void _explorerDirPath(UpDroidMessage um) {
    workspacePath = um.body;
    _showNodesController();
  }

  void _showWorkspacesController() {
    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // TODO: disable Workspaces button.
    controller = new UpDroidWorkspaces(id, workspacePath, view, mailbox);
  }

  void _showNodesController() {
    if (controller != null) {
      controller.cleanUp();
      controller = null;
    }

    // TODO: disable Nodes button.
    controller = new UpDroidNodes(id, workspacePath, view, mailbox);
  }

  //\/\/ Handler Helpers /\/\//


  void cleanUp() {

  }
}

abstract class ExplorerController {
  void registerMailbox();
  void registerEventHandlers();
  void cleanUp();
}