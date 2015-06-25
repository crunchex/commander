library updroid_explorer_workspaces;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'package:dnd/dnd.dart';
import 'package:path/path.dart' as pathLib;

import '../../../context_menu.dart';
import '../../../mailbox.dart';
import '../../panel_controller.dart';
import '../explorer.dart';

part 'workspaces_view.dart';
part 'fs_entity.dart';

class UpDroidWorkspaces implements ExplorerController {
  PanelView _view;
  WorkspacesView _workspacesView;
  Mailbox _mailbox;

  AnchorElement _buildPackagesButton;
  AnchorElement _cleanButton;
//  AnchorElement _uploadButton;

  Dropzone dzRecycle;

  Map<String, FileSystemEntity> entities = {};
  String workspacePath;
  Set<StreamSubscription> _listenersToCleanUp;

  UpDroidWorkspaces(int id, this.workspacePath, PanelView view, Mailbox mailbox, List<AnchorElement> actionButtons) {
    _view = view;
    _mailbox = mailbox;
    _buildPackagesButton = actionButtons[0];
    _cleanButton = actionButtons[1];

    registerMailbox();

    _listenersToCleanUp = new Set<StreamSubscription>();

    WorkspacesView.createWorkspacesView(id, _view.content).then((workspacesView) {
      _workspacesView = workspacesView;

      dzRecycle = new Dropzone(_workspacesView.recycle);

      _mailbox.ws.send('[[REQUEST_WORKSPACE_CONTENTS]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_CONTENTS', _workspaceContents);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'ADD_UPDATE', _addUpdate);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'REMOVE_UPDATE', _removeUpdate);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'BUILD_COMPLETE', _buildComplete);
  }

  void registerEventHandlers() {
    _listenersToCleanUp.add(_cleanButton.onClick.listen((e) => _mailbox.ws.send('[[WORKSPACE_CLEAN]]')));
    _listenersToCleanUp.add(_buildPackagesButton.onClick.listen((e) => _buildPackages()));
//    _uploadButton.onClick.listen((e) => new UpDroidGitPassModal(cs));
  }

  /// Handles an Explorer add update for a single file.
  void _addUpdate(UpDroidMessage um) => _addFileSystemEntity(um.body);

  void _addFileSystemEntity(String data) {
    List<String> split = data.split(':');
    bool isDir = split[0] == 'D' ? true : false;
    String path = data.split(':')[1];

    // Don't do anything if the entity is already in the system or if anything
    // in its path is hidden.
    bool hiddenFolderInPath = false;
    for (String pathSegment in path.split('/')) {
      if (pathSegment.startsWith('.')) {
        hiddenFolderInPath = true;
        break;
      }
    }
    if (entities.containsKey(path) || hiddenFolderInPath) return;

    // Recursively add a parent that isn't in the system yet.
    String parentPath = FileSystemEntity.getParentFromPath(path, workspacePath);
    if (parentPath != null && !entities.containsKey(parentPath)) {
      _addFileSystemEntity('D:$parentPath');
    }

    FileSystemEntity entity;
    if (isDir) {
      entity = new FolderEntity(path, workspacePath, _mailbox.ws, _deselectAllEntities);
    } else {
      entity = new FileEntity(path, workspacePath, _mailbox.ws, _deselectAllEntities);
    }
    entities[entity.path] = entity;

    // Special case for the workspace src directory (root node).
    if (entity.parent == null) {
      _workspacesView.uList.children.add(entity.view.element);
      FolderView folderView = entity.view;
      folderView.toggleExpansion();
      return;
    }

    // If current file is a CMakeLists.txt, tell its parent that it's a package folder.
    if (entity.name == 'CMakeLists.txt') entities[entity.parent].isPackage = true;

//    _insertView(entities[entity.parent], entity);

    FolderView parentFolder = entities[entity.parent].view;
    parentFolder.uElement.children.add(entity.view.element);
  }

  /// TODO: fix this... not working.
  /// Inserts a view into the DOM based on path alphabetical ordering.
  void _insertView(FolderEntity parentFolder, FileSystemEntity entity) {
    List<String> siblingKeys = entities.keys.where((String key) => key.contains(parentFolder.path));
    List<String> siblingPaths = new List<String>.from(siblingKeys);
    siblingPaths.sort();

    String olderSiblingPath;
    for (String siblingPath in siblingPaths) {
      olderSiblingPath = siblingPath;
      if (siblingPath.compareTo(entity.path) > 0) break;
    }

    String olderSiblingName = olderSiblingPath.split('/').last;
    LIElement olderSiblingView;

    FolderView parentView = parentFolder.view;
    print('looking for $olderSiblingName');
    parentView.uElement.children.forEach((LIElement childLi) {
      String childName = childLi.firstChild.lastChild.text;
      print('$childName');
      if (childName == olderSiblingName) {
        print('found view by name $olderSiblingName');
        olderSiblingView = childLi;
        return;
      }
    });

    parentView.uElement.insertBefore(entity.view.element, olderSiblingView);
  }

  /// Handles an Explorer remove update for a single file.
  void _removeUpdate(UpDroidMessage um) => _removeFileSystemEntity(um.body);

  void _removeFileSystemEntity(String data) {
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
  void _workspaceContents(UpDroidMessage um) {
    List<String> fileStrings = JSON.decode(um.body);

    _workspacesView.uList.innerHtml = '';

    for (String rawString in fileStrings) {
      _addFileSystemEntity(rawString);
    }
  }

  void _deselectAllEntities() {
    entities.values.forEach((FileSystemEntity entity) => entity.deselect());
  }

  void _buildPackages() {
    List<String> packageBuildList = [];
    entities.values.forEach((FolderEntity entity) {
      if (entity.isPackage && entity.selected) {
        entity.toggleBuildingIndicator();
        packageBuildList.add(entity.path);
      }
    });

    _mailbox.ws.send('[[BUILD_PACKAGES]]' + JSON.encode(packageBuildList));

//    _mailbox.ws.send('[[WORKSPACE_BUILD]]');
  }

  void _buildComplete(UpDroidMessage um) {
    List<String> entityPaths = JSON.decode(um.body);
    entityPaths.forEach((String entityPath) {
      FolderEntity package = entities[entityPath];
      package.toggleBuildingIndicator();
    });
  }

  void cleanUp() {
    _listenersToCleanUp.forEach((StreamSubscription s) => s.cancel());

    entities.values.forEach((FileSystemEntity f) => f.cleanUp());
    entities = null;

    _workspacesView.cleanUp();
  }
}

///// Sets up a [Draggable] for the existing [LIElement] to handle file open and delete.
//void dragSetup(LIElement li, FileSystemEntity file) {
//  // Create a new draggable using the current element as
//  // the visual element (avatar) being dragged.
//  Draggable d = new Draggable(li, avatarHandler: new AvatarHandler.clone());
//
//  // Dragging through nested dropzones appears to be glitchy.
//  d.onDragStart.listen((event) {
//    d.avatarHandler.avatar.children.first.classes.remove('highlighted');
//    if (pathLib.dirname(li.dataset['path']) != workspacePath) _workspacesView.drop.classes.add('file-drop-ondrag');
//    _workspacesView.recycle.classes.add('recycle-ondrag');
//    ElementList<SpanElement> spanList = querySelectorAll('.glyphicons-folder-open');
//    ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
//    for (SpanElement span in spanList) {
//      span.classes.add('span-ondrag');
//    }
//    for (SpanElement span in closedList) {
//      span.classes.add('span-ondrag');
//    }
//    if (!file.isDirectory) {
//      cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_ADD', body: 'updroideditor-ondrag'));
//    }
//  });
//
//  d.onDragEnd.listen((event) {
//    _workspacesView.drop.classes.remove('file-drop-ondrag');
//    _workspacesView.recycle.classes.remove('recycle-ondrag');
//    ElementList<SpanElement> spanList = querySelectorAll('.glyphicons-folder-open');
//    ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
//    for (SpanElement span in spanList) {
//      span.classes.remove('span-ondrag');
//    }
//    for (SpanElement span in closedList) {
//      span.classes.remove('span-ondrag');
//    }
//    if (!file.isDirectory) {
//      cs.add(new CommanderMessage('UPDROIDEDITOR', 'CLASS_REMOVE', body: 'updroideditor-ondrag'));
//    }
//  });
//}