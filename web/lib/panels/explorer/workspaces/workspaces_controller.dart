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

      _mailbox.ws.send('[[REQUEST_WORKSPACE_CONTENTS]]');

      registerEventHandlers();
    });
  }

  void registerMailbox() {
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'WORKSPACE_CONTENTS', workspaceContents);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'ADD_UPDATE', addUpdate);
    _mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'REMOVE_UPDATE', removeUpdate);
  }

  void registerEventHandlers() {
    _cleanButton.onClick.listen((e) => _mailbox.ws.send('[[WORKSPACE_CLEAN]]'));
    _buildButton.onClick.listen((e) => _mailbox.ws.send('[[WORKSPACE_BUILD]]'));
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
    List<String> split = data.split(':');
    bool isDir = split[0] == 'D' ? true : false;
    String path = data.split(':')[1];

    // Don't do anything if the entity is already in the system.
    if (entities.containsKey(path)) return;

    // Recursively add a parent that isn't in the system yet.
    String parentPath = FileSystemEntity.getParentFromPath(path, workspacePath);
    if (parentPath != null && !entities.containsKey(parentPath)) {
      addFileSystemEntity('D:$parentPath');
    }

    FileSystemEntity entity;
    if (isDir) {
      entity = new FolderEntity(path, workspacePath, _mailbox.ws);
    } else {
      entity = new FileEntity(path, workspacePath, _mailbox.ws);
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
  void workspaceContents(UpDroidMessage um) {
    List<String> fileStrings = JSON.decode(um.body);

    _workspacesView.uList.innerHtml = '';

    for (String rawString in fileStrings) {
      addFileSystemEntity(rawString);
    }
  }

  void cleanUp() {
    entities.values.forEach((FileSystemEntity f) => f.cleanUp());
    entities = null;
    _workspacesView.cleanUp();
  }
}