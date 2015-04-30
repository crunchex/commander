library updroid_explorer;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

import 'package:dnd/dnd.dart';
import "package:path/path.dart" as pathLib;

import 'tab.dart';
import 'lib/updroid_message.dart';
import 'lib/explorer_helper.dart';

/// [UpDroidExplorer] manages the data for the file explorer on the client
/// side and all associated views. It also facilitates file operation requests
/// to the server side.
class UpDroidExplorer extends UpDroidTab{
  static const String className = 'UpDroidExplorer';

  // Make dynamic
  int expNum;
  DivElement expCon;
  DivElement explorer;

  String workspacePath;
  DivElement currentSelected;
  String currentSelectedPath;

  LIElement title;
  DivElement editorDiv;
  ButtonElement controlToggle;
  DivElement controlPanel;
  SpanElement newFile;
  SpanElement newFolder;
  DivElement newFileDrop;
  DivElement rootlineContainer;
  ParagraphElement recycle;
  LIElement fileName;

  Draggable dragNewFile;
  Draggable dragNewFolder;

  Dropzone dzRootLineContainer;
  Dropzone dzRecycle;
  StreamSubscription outsideClickListener;
  StreamSubscription controlLeave;
  Map expRefs;
  Map editors = {};
  Map editorListeners = {};
  Map fileInfo = {};
  Map pathToFile = {};
  Map ulInfo = {};

  WebSocket ws;
  StreamController<CommanderMessage> cs;

  UpDroidExplorer(StreamController<CommanderMessage> cs, num, name) {
    expNum = num;
    this.cs = cs;
    expRefs = createExplorer(num, name);
    expCon = expRefs['explorersDiv'];
    explorer = expRefs['explorer'];

    newFile = expRefs['file'];
    newFileDragSetup();

    newFolder = expRefs['folder'];
    newFolderDragSetup();

    title = expRefs['title'];
    controlToggle = expRefs['controlToggle'];
    controlPanel = expRefs['control'];
    newFileDrop = expRefs['drop'];
    rootlineContainer = expRefs['hrContainer'];
    dzRootLineContainer = new Dropzone(rootlineContainer);
    recycle = expRefs['recycle'];
    dzRecycle = new Dropzone(recycle);

    // Create the server <-> client [WebSocket].
    // Port 12060 is the default port that UpDroid uses.
    String url = window.location.host;
    url = url.split(':')[0];
    ws = new WebSocket('ws://' + url + ':12060/explorer/${expNum.toString()}');

    registerExplorerEventHandlers();
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CONNECTED':
        break;

      case 'DISCONNECTED':
        break;

      case 'EDITOR_READY':
        var num = m.body[0]; // Editor num
        var dropDiv = m.body[1];

        var dzEditor = new Dropzone(dropDiv);
        editorListeners.putIfAbsent(
            dzEditor, () => createEditorListeners(dzEditor));
        editors.putIfAbsent(dzEditor, () => num);
        cs.add(new CommanderMessage('EDITOR', 'PASS_EDITOR_INFO',
            body: [num, dzEditor]));
        break;

      case 'REQUEST_PARENT_PATH':
        if(!explorer.classes.contains('hidden')) {
          cs.add(new CommanderMessage('EDITOR', 'PARENT_PATH',
              body: currentSelectedPath));
        }
        break;

      case 'REMOVE_EDITOR':
        editors.remove(m.body);
        for (var stream in editorListeners[m.body]) {
          stream.cancel();
        }
        editorListeners.remove(m.body);
        break;

      default:
        print('Explorer error: unrecognized message type: ' + m.type);
    }
    // Dragging through nested dropzones appears to be glitchy

  }

  // TODO: cancel when inactive
  List<StreamSubscription> createEditorListeners(Dropzone dzEditor) {
    var enter = dzEditor.onDragEnter.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD',
            body: 'editor-entered'));
      }
    });

    var leave = dzEditor.onDragLeave.listen((e) => cs.add(new CommanderMessage(
        'EDITOR', 'CLASS_REMOVE', body: 'editor-entered')));

    var drop = dzEditor.onDrop.listen((e) {
      if(!explorer.classes.contains('hidden')) {
        var isDir = e.draggableElement.dataset['isDir'];
        if (isDir == 'false') {
          var num = editors[dzEditor];
          cs.add(new CommanderMessage('EDITOR', 'OPEN_FILE',
              body: [num, getPath(e.draggableElement)]));
      }
      }
    });
    return [enter, leave, drop];
  }

  /// Sets up the event handlers for the file explorer. Mostly mouse events.
  registerExplorerEventHandlers() {
    cs.stream
        .where((m) => m.dest == 'EXPLORER' || m.dest == 'ALL')
        .listen((m) => processMessage(m));

    ws.onOpen.listen((e) => ws.send('[[EXPLORER_DIRECTORY_PATH]]'));

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_PATH')
        .listen((um) {
      workspacePath = um.body;
      ws.send('[[INITIAL_DIRECTORY_LIST]]');
    });

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'INITIAL_DIRECTORY_LIST')
        .listen((um) => initialDirectoryList(um.body));

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_LIST')
        .listen((um) => generateDirectoryList(um.body));

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_REFRESH')
        .listen((um) => refreshPage(um.body));

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_ADD')
        .listen((um) => addUpdate(um.body));

    ws.onMessage
        .transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_REMOVE')
        .listen((um) => removeUpdate(um.body));

    controlToggle.onClick.listen((e) => showControl());

    newFileDrop.onClick.listen((e) {
      if (currentSelected != null) {
        currentSelected.classes.remove('highlighted');
      }
      currentSelected = null;
      currentSelectedPath = workspacePath;
    });

    dzRootLineContainer.onDragEnter
        .listen((e) => newFileDrop.classes.add('file-drop-entered'));
    dzRootLineContainer.onDragLeave
        .listen((e) => newFileDrop.classes.remove('file-drop-entered'));

    dzRootLineContainer.onDrop.listen((e) {
      if (e.draggableElement.className.contains('explorer-li')) {

        // The draggable is an existing file/folder.

        var currentPath = getPath(e.draggableElement);
        LIElement item = pathToFile[currentPath];
        var newPath = '$workspacePath/${getName(e.draggableElement)}';
        bool duplicate = false;

        // Check for duplicate file name
        if (pathToFile.containsKey(newPath)) {
          duplicate = true;
        }

        bool alert = false;

        if (duplicate == true) {
          alert = true;
          window.alert("Cannot move here, filename already exists");
        }

        if (e.draggableElement.dataset['isDir'] == 'true') {

          // Avoid an exception thrown when the new name already exists or dragging to same folder.

          if (currentPath != newPath && alert == false) {
            if (item.lastChild.hasChildNodes() == false) {
              ws.send(
                  '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              var name = getName(e.draggableElement);
              removeFileData(item, currentPath);
              newElementFromFile(new SimpleFile.fromPath(
                  workspacePath + '/' + name, workspacePath, true));
              item.remove();
            } // TODO: Preserve structure
            else if (checkContents(item) == true) {
              ws.send(
                  '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              removeSubFolders(item);
              removeFileData(e.draggableElement, currentPath);
              ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
              item.remove();
            } else {
              ws.send(
                  '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              removeFileData(e.draggableElement, currentPath);
              item.remove();
            }
          }
        } else {
          if (currentPath != newPath && alert == false) {
            ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
          }
        }
      } else if (e.draggableElement.classes.contains('file')) {
        ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath);
      }

      else {
        ws.send('[[EXPLORER_NEW_FOLDER]]' + workspacePath + '/untitled');
      }
    });

    newFolder.onDoubleClick.listen((e) {
      if(currentSelectedPath == null) ws.send('[[EXPLORER_NEW_FOLDER]]' + workspacePath + '/untitled');
      else{
        ws.send('[[EXPLORER_NEW_FOLDER]]' + currentSelectedPath + '/untitled');
        }
    });

    newFile.onDoubleClick.listen((e) {
      if(currentSelectedPath == null) ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath);
      else{
        ws.send('[[EXPLORER_NEW_FILE]]' + currentSelectedPath);
      }
    });

    // TODO: cancel when inactive
    dzRecycle.onDragEnter.listen((e) => recycle.classes.add('recycle-entered'));
    dzRecycle.onDragLeave
        .listen((e) => recycle.classes.remove('recycle-entered'));

    dzRecycle.onDrop.listen((e) {
      if (!explorer.classes.contains('hidden')) {
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

        ws.send('[[EXPLORER_DELETE]]' + path);
      }
    });
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
      files.add(new SimpleFile.fromDirectoryList(entity, workspacePath));
    }

    return files;
  }

  /// Shows control panel
  void showControl () {
    if(expCon != null) expCon.classes.add('hidden');
    controlPanel.classes.remove('hidden');
    controlLeave = title.onClick.listen((e){
      hideControl();
      controlLeave.cancel();
    });
  }

  void hideControl () {
    controlPanel.classes.add('hidden');
    expCon.classes.remove('hidden');
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
    LIElement li = new LIElement();
    li
      ..dataset['name'] = file.name
      ..dataset['path'] = file.path
      ..dataset['exp'] = expNum.toString()
      ..dataset['isDir'] = file.isDirectory.toString()
      ..draggable = true
      ..classes.add('explorer-li');

    fileInfo.putIfAbsent(li, () => [file.name, file.path]);
    pathToFile.putIfAbsent(file.path, () => li);
    // Create a span element for the glyphicon
    SpanElement glyphicon = new SpanElement();
    SpanElement glyph = new SpanElement();
    glyph.classes
        .addAll(['glyphicon', 'glyphicon-folder-close', "list-folder"]);
    var glyphType =
        (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
    glyphicon.classes.addAll(['glyphicon', glyphType]);
    dropSetup(glyphicon, file);
    dropSetup(glyph, file);
    DivElement fileContainer = new DivElement();
    setupHighlighter(fileContainer);
    fileContainer.classes.add('file-container');
    li.children.add(fileContainer);
    fileContainer.children.add(glyphicon);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement();
    filename
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
          var newPath =
              '${getPath(span.parent.parent)}/${getName(e.draggableElement)}';
          bool duplicate;
          LIElement item = pathToFile[currentPath];

          // Check for duplicate file name
          pathToFile
              .containsKey(newPath) ? duplicate = true : duplicate = false;
          bool alert = false;
          if (duplicate == true) {
            alert = true;
          }

          // The draggable is an empty folder

          if (e.draggableElement.dataset['isDir'] == 'true') {
            bool send = true;
            if (getPath(span.parent.parent)
                .contains(getPath(e.draggableElement))) {
              send = checkNested(
                  getPath(span.parent.parent), getPath(e.draggableElement));
            }
            // Avoid an exception thrown when the new name already exists or dragging to same folder.

            if (currentPath != newPath && duplicate == false && send == true) {
              if (item.lastChild.hasChildNodes() == false) {
                ws.send(
                    '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                var name = getName(e.draggableElement);
                removeFileData(e.draggableElement, currentPath);
                newElementFromFile(new SimpleFile.fromPath(
                    getPath(span.parent.parent) + '/' + name, workspacePath,
                    true));
                item.remove();
              } else if (checkContents(item) == true) {
                ws.send(
                    '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                removeSubFolders(item);
                removeFileData(e.draggableElement, currentPath);
                ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
                item.remove();
              } else {
                ws.send(
                    '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                item.remove();
                removeFileData(e.draggableElement, currentPath);
              }
            }
          } else {
            if (currentPath != newPath &&
                duplicate == false &&
                !getPath(span.parent.parent)
                    .contains(getPath(e.draggableElement))) {
              ws.send(
                  '[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
            }
          }

          if (alert == true && item != duplicate) {
            window.alert("Cannot move here, file name already exists");
          }
        } else if (e.draggableElement.classes.contains('file')) {
          ws.send('[[EXPLORER_NEW_FILE]]' + getPath(span.parent.parent));
        } else {
          ws.send('[[EXPLORER_NEW_FOLDER]]' +
              getPath(span.parent.parent) +
              '/untitled');
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
          ws.send('[[EXPLORER_DIRECTORY_LIST]]');
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
            ws.send('[[EXPLORER_RENAME]]' + file.path + ':divider:' + newPath);
            if (folder == true) {
              removeFileData(li, file.path);
            }
          }

          // TODO: Create a overwrite option in case of existing file name

          if (duplicate == true) {
            if (duplicate == li) {
              ws.send('[[EXPLORER_DIRECTORY_LIST]]');
            } else {
              window.alert("File name already exists");
              ws.send('[[EXPLORER_DIRECTORY_LIST]]');
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
              ws.send('[[EXPLORER_DIRECTORY_LIST]]');
            }
          }

          if (refresh == true) {
            removeSubFolders(li);
            ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
          }

          // Create a folder icon if the item renamed was an empty folder

          if (file.isDirectory == true &&
              li.lastChild.hasChildNodes() == false) {
            newElementFromFile(
                new SimpleFile.fromPath(newPath, workspacePath, true));
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

  /// Sets up a [Draggable] for the [newFile] to handle the drag event.
  void newFileDragSetup() {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d =
        new Draggable(newFile, avatarHandler: new AvatarHandler.clone());

    // Highlight valid dropzones: rootline, editor, any workspace folder.
    d.onDragStart.listen((event) {
      newFileDrop.classes.add('file-drop-ondrag');
      cs.add(
          new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.add('span-ondrag');
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE',
          body: 'editor-ondrag'));
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.remove('span-ondrag');
      }
    });
  }

  /// Sets up a [Draggable] for the [newFolder] to handle the drag event.
  void newFolderDragSetup() {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d =
        new Draggable(newFolder, avatarHandler: new AvatarHandler.clone());

    // Highlight valid dropzones: rootline, any workspace folder.
    d.onDragStart.listen((event) {
      newFileDrop.classes.add('file-drop-ondrag');
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.add('span-ondrag');
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.remove('span-ondrag');
      }
    });
  }

  /// Sets up a [Draggable] for the existing [LIElement] to handle file open and delete.
  void dragSetup(LIElement li, SimpleFile file) {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d = new Draggable(li, avatarHandler: new AvatarHandler.clone());

    // Dragging through nested dropzones appears to be glitchy.
    d.onDragStart.listen((event) {
      d.avatarHandler.avatar.children.first.classes.remove('highlighted');
      newFileDrop.classes.add('file-drop-ondrag');
      recycle.classes.add('recycle-ondrag');
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.add('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(
            new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      recycle.classes.remove('recycle-ondrag');
      ElementList<SpanElement> spanList =
          querySelectorAll('.glyphicon-folder-open');
      ElementList<SpanElement> closedList = querySelectorAll('.list-folder');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
      for (SpanElement span in closedList) {
        span.classes.remove('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE',
            body: 'editor-ondrag'));
      }
    });
  }

  /// Handles an Explorer add update for a single file.
  void addUpdate(String path) {
    SimpleFile sFile = new SimpleFile.fromPath(path, workspacePath, false);
    var parentPath = pathLib.dirname(sFile.path);

    // Try to detect the parent, and if it doesn't exist then create the element for it.
    String curPath = '';

    // Iterate through the path checking to see if the folder exists
    var split = parentPath
        .replaceFirst(pathLib.normalize(workspacePath), '')
        .split('/');
    for (int i = 1; i < split.length; i++) {
      curPath += split[i];
      LIElement curLi = pathToFile['${pathLib.join(workspacePath, curPath)}'];
      if (curLi == null &&
          pathLib.join(workspacePath, curPath) != workspacePath) {
        newElementFromFile(new SimpleFile.fromPath(
                pathLib.join(workspacePath, curPath), workspacePath, true))
            .then((result) {});
      }
      if (i != split.length - 1) {
        curPath += '/';
      }
    }
    newElementFromFile(sFile);
  }

  /// Handles an Explorer remove update for a single file.

  void removeUpdate(String path) {
    LIElement li = pathToFile[path];

    // Case to deal with removeUpdate grabbing null objects when items are renamed
    if (li == null) {} else {
      li.remove();
    }
    removeFileData(li, path);
  }

  /// Sets up a new HTML element from a SimpleFile.
  Future newElementFromFile(SimpleFile file, [bool expanded]) {
    Completer completer = new Completer();
    completer.complete(true);

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
      dirElement = querySelector('#explorer-body-$expNum');
      dirElement.append(li);
    } else if (!file.path.contains('/.') &&
        !file.path.contains('CMakeLists.txt')) {
      dirElement = ulInfo[pathLib.dirname(file.path)];
      dirElement.append(li);
    }

    return completer.future;
  }

  /// First Directory List Generation

  void initialDirectoryList(String raw) {
    var files = fileList(raw);

    UListElement explorer = querySelector('#explorer-body-$expNum');
    explorer.innerHtml = '';

    for (SimpleFile file in files) {
      newElementFromFile(file, false);
    }
  }

  /// Redraws all file explorer views.
  void generateDirectoryList(String raw) {
    var files = fileList(raw);
    var folderStateList = {};
    for (var file in files) {
      if (file.isDirectory == true) {
        LIElement folder = pathToFile[file.path];
        if (folder != null) {
          folder.dataset['expanded'] == "true"
              ? folderStateList[file] = true
              : folderStateList[file] = false;
        }
      }
    }

    ulInfo.clear();
    fileInfo.clear();
    pathToFile.clear();

    // Set the explorer list to empty for a full refresh.
    UListElement explorer = querySelector('#explorer-body-$expNum');
    explorer.innerHtml = '';

    for (SimpleFile file in files) {
      file.isDirectory == true
          ? newElementFromFile(file, folderStateList[file])
          : newElementFromFile(file);
    }
  }

  /// Update the explorer view in case of nested folder movement to cover for empty folders

  void refreshPage(String raw) {
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
}

/// Container class that extracts data from the raw file text passed in from
/// the server over [WebSocket]. Primarily used for generating the HTML views
/// in the file explorer that represent the filesystem.
class SimpleFile {
  String name;
  String parentDir;
  String path;
  bool isDirectory;

  SimpleFile.fromDirectoryList(String raw, String prefix) {
    String workingString = stripFormatting(raw, prefix);
    getData(workingString);
  }

  SimpleFile.fromPath(String raw, String prefix, bool isDir) {
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
    name = pathList[pathList.length - 1];
    if (pathList.length > 1) {
      parentDir = pathList[pathList.length - 2];
    } else {
      parentDir = '';
    }
  }
}
