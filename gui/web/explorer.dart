part of updroid_client;

/// [UpDroidExplorer] manages the data for the file explorer on the client
/// side and all associated views. It also facilitates file operation requests
/// to the server side.
class UpDroidExplorer {
  WebSocket ws;
  StreamController<CommanderMessage> cs;
  String workspacePath;

  DivElement editorDiv;
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
  Dropzone dzEditor;
  StreamSubscription outsideClickListener;

  UpDroidExplorer(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;

    newFile = querySelector('#file');
    newFileDragSetup();

    newFolder = querySelector('#folder');
    newFolderDragSetup();

    newFileDrop = querySelector('#new-file-drop');
    rootlineContainer = querySelector('#file-explorer-hr-container');
    dzRootLineContainer = new Dropzone(rootlineContainer);
    recycle = querySelector('#recycle');
    dzRecycle = new Dropzone(recycle);

    editorDiv = querySelector('#editor');
    dzEditor = new Dropzone(editorDiv);
    fileName = querySelector('#filename');

    registerExplorerEventHandlers();

    cs.add(new CommanderMessage('CLIENT', 'EXPLORER_READY'));
  }

  /// Process messages according to the type.
  void processMessage(CommanderMessage m) {
    switch (m.type) {
      case 'CONNECTED':
        ws.send('[[EXPLORER_DIRECTORY_PATH]]');
        break;

      case 'DISCONNECTED':
        break;

      default:
        print('Explorer error: unrecognized message type: ' + m.type);
    }
  }

  /// Sets up the event handlers for the file explorer. Mostly mouse events.
  registerExplorerEventHandlers() {
    cs.stream.where((m) => m.dest == 'EXPLORER' || m.dest == 'ALL').listen((m) => processMessage(m));

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'EXPLORER_DIRECTORY_PATH').listen((um) {
      workspacePath = um.body;
      ws.send('[[INITIAL_DIRECTORY_LIST]]');
    });

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'INITIAL_DIRECTORY_LIST').listen((um) => initialDirectoryList(um.body));

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'EXPLORER_DIRECTORY_LIST').listen((um) => generateDirectoryList(um.body));

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'EXPLORER_DIRECTORY_REFRESH').listen((um) => refreshPage(um.body));

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'EXPLORER_ADD').listen((um) => addUpdate(um.body));

    ws.onMessage.transform(updroidTransformer).where((um) => um.header == 'EXPLORER_REMOVE').listen((um) => removeUpdate(um.body));

    dzRootLineContainer.onDragEnter.listen((e) => newFileDrop.classes.add('file-drop-entered'));
    dzRootLineContainer.onDragLeave.listen((e) => newFileDrop.classes.remove('file-drop-entered'));

    dzRootLineContainer.onDrop.listen((e) {
      if (e.draggableElement.className.contains('explorer-li')) {


        // The draggable is an existing file/folder.

        var currentPath = e.draggableElement.dataset['path'];
        LIElement item = querySelector('[data-path="$currentPath"]');
        var newPath = '$workspacePath/${e.draggableElement.dataset['trueName']}';

        // Check for duplicate file name
        LIElement duplicate = querySelector('[data-path="$workspacePath/${e.draggableElement.dataset['trueName']}"]');
        bool alert = false;

        if (duplicate != null && duplicate != item) {
          alert = true;
          window.alert("Cannot move here, filename already exists");
        }

        if (e.draggableElement.dataset['isDir'] == 'true') {

          // Avoid an exception thrown when the new name already exists or dragging to same folder.

          if (currentPath != newPath && alert == false) {
            if (item.lastChild.hasChildNodes() == false) {
              ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              newElementFromFile(new SimpleFile.fromPath(workspacePath + '/' + e.draggableElement.dataset['trueName'], workspacePath, true));
              item.remove();
            } // TODO: Preserve structure
            else if (checkContents(item) == true) {
              ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
              item.remove();
            } // TODO: Preserve structure
            else {
              ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
              item.remove();
            }
          }
        } else {
          if (currentPath != newPath && alert == false) {
            ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
          }
        }

      } else if (e.draggableElement.id == 'file') {
        ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath);
      } else {
        ws.send('[[EXPLORER_NEW_FOLDER]]' + workspacePath + '/untitled');
      }
    });

    newFolder.onDoubleClick.listen((e) {
      ws.send('[[EXPLORER_NEW_FOLDER]]' + workspacePath + '/untitled');
    });

    newFile.onDoubleClick.listen((e) {
      ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath);
    });

    dzRecycle.onDragEnter.listen((e) => recycle.classes.add('recycle-entered'));
    dzRecycle.onDragLeave.listen((e) => recycle.classes.remove('recycle-entered'));

    dzRecycle.onDrop.listen((e) {
      var path = e.draggableElement.dataset['path'];

      // Draggable is an empty folder
      if (e.draggableElement.dataset['isDir'] == 'true') {
        LIElement selectedFolder = querySelector('[data-path="${e.draggableElement.dataset['path']}"');
        selectedFolder.remove();
      }

      ws.send('[[EXPLORER_DELETE]]' + path);
    });

    // Dragging through nested dropzones appears to be glitchy
    dzEditor.onDragEnter.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-entered'));
      }
    });

    dzEditor.onDragLeave.listen((e) => cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE', body: 'editor-entered')));

    dzEditor.onDrop.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        cs.add(new CommanderMessage('EDITOR', 'OPEN_FILE', body: e.draggableElement.dataset['path']));
        fileName.text = e.draggableElement.dataset['trueName'];
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


  /// Returns a generated [LIElement] with inner HTML based on
  /// the [SimpleFile]'s contents.
  LIElement generateLiHtml(file, [expanded]) {
    LIElement li = new LIElement();
    li
        ..dataset['name'] = removeSpaces(file.name)
        ..dataset['trueName'] = (file.name)
        ..dataset['path'] = file.path
        ..dataset['isDir'] = file.isDirectory.toString()
        ..draggable = true
        ..classes.add('explorer-li');

    // Create a span element for the glyphicon
    SpanElement glyphicon = new SpanElement();
    SpanElement glyph = new SpanElement();
    glyph.classes.addAll(['glyphicon', 'glyphicon-folder-close', "list-folder"]);
    var glyphType = (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
    glyphicon.classes.addAll(['glyphicon', glyphType]);
    dropSetup(glyphicon, file);
    dropSetup(glyph, file);
    li.children.add(glyphicon);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement();
    filename
        ..classes.add('filename')
        ..text = file.name;
    li.children.add(filename);

    if (file.isDirectory) {
      li.dataset['expanded'] = 'false';
      UListElement ul = new UListElement();
      ul
          ..dataset['name'] = 'explorer-ul-${removeSpaces(file.name)}'
          ..dataset['path'] = removeSpaces(file.path)
          ..classes.addAll(['explorer', 'explorer-ul']);
      li.children.add(ul);

      ul.classes.add("explorer-hidden");
      glyphicon.replaceWith(glyph);
      li.dataset['expanded'] = 'false';

      if (expanded == true) {
        glyph.replaceWith(glyphicon);
        li.dataset['expanded'] = 'true';
        ul.classes.remove("explorer-hidden");
      }

      glyphicon.onClick.listen((e) {
        ul.classes.add("explorer-hidden");
        glyphicon.replaceWith(glyph);
        li.dataset['expanded'] = 'false';
      });

      glyph.onClick.listen((e) {
        glyph.replaceWith(glyphicon);
        li.dataset['expanded'] = 'true';
        ul.classes.remove("explorer-hidden");
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
          var currentPath = e.draggableElement.dataset['path'];
          var newPath = '${span.parent.dataset['path']}/${e.draggableElement.dataset['trueName']}';
          LIElement item = querySelector('[data-path="$currentPath"]');

          // Check for duplicate file name
          LIElement duplicate = querySelector('[data-path="${span.parent.dataset['path']}/${e.draggableElement.dataset['trueName']}"]');
          bool alert = false;
          if (duplicate != null) {
            alert = true;
          }

          // The draggable is an empty folder

          if (e.draggableElement.dataset['isDir'] == 'true') {

            bool send = true;
            if (span.parent.dataset['path'].contains(e.draggableElement.dataset['path'])) {
              send = checkNested(span.parent.dataset['path'], e.draggableElement.dataset['path']);
            }
            // Avoid an exception thrown when the new name already exists or dragging to same folder.

            if (currentPath != newPath && duplicate == null && send == true) {
              if (item.lastChild.hasChildNodes() == false) {
                ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                item.remove();
                newElementFromFile(new SimpleFile.fromPath(span.parent.dataset['path'] + '/' + e.draggableElement.dataset['trueName'], workspacePath, true));
              } else if (checkContents(item) == true) {
                ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
                item.remove();
              } else {
                ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
                item.remove();
              }
            }
          } else {
            if (currentPath != newPath && duplicate == null && !span.parent.dataset['path'].contains(e.draggableElement.dataset['path'])) {
              ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
            }
          }

          if (alert == true && item != duplicate) {
            window.alert("Cannot move here, file name already exists.");
          }

        } else if (e.draggableElement.id == 'file') {
          ws.send('[[EXPLORER_NEW_FILE]]' + span.parent.dataset['path']);
        } else {
          ws.send('[[EXPLORER_NEW_FOLDER]]' + span.parent.dataset['path'] + '/untitled');
        }
      });
    }
  }

  /// Handles file renaming with a double-click event.
  void renameEventHandler(LIElement li, SimpleFile file) {
    bool refresh = false;
    bool renameFinish = false;

    if (li.dataset['isDir'] == 'true' && checkContents(li) == true) {
      refresh = true;
    }

    if (!li.className.contains('editing')) {
      li.classes.add('editing');

      InputElement input = new InputElement();
      input.value = '${li.dataset['trueName']}';

      Element outside = querySelector('.container-fluid');

      outsideClickListener = outside.onClick.listen((e) {
        if (e.target != input && renameFinish == false) {
          ws.send('[[EXPLORER_DIRECTORY_LIST]]');
          outsideClickListener.cancel();
        }
      });

      // TODO: this only works in Chromium

      input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          renameFinish = true;
          var newPath = pathLib.join(pathLib.dirname(file.path), input.value);

          LIElement duplicate = querySelector("[data-path='$newPath']");
          if (duplicate == null) {
            ws.send('[[EXPLORER_RENAME]]' + file.path + ':divider:' + newPath);
          }

          // TODO: Create a overwrite option in case of existing file name

          if (duplicate != null) {
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
            ws.send('[[EXPLORER_DIRECTORY_REFRESH]]');
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
      li.children[1] = input;
      input.focus();
      input.select();
    }
  }

  /// Sets up a [Draggable] for the [newFile] to handle the drag event.
  void newFileDragSetup() {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d = new Draggable(newFile, avatarHandler: new AvatarHandler.clone());

    // Highlight valid dropzones: rootline, editor, any workspace folder.
    d.onDragStart.listen((event) {
      newFileDrop.classes.add('file-drop-ondrag');
      cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
        span.classes.add('span-ondrag');
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE', body: 'editor-ondrag'));
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
        span.classes.remove('span-ondrag');
      }
    });
  }

  /// Sets up a [Draggable] for the [newFolder] to handle the drag event.
  void newFolderDragSetup() {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d = new Draggable(newFolder, avatarHandler: new AvatarHandler.clone());

    // Highlight valid dropzones: rootline, any workspace folder.
    d.onDragStart.listen((event) {
      newFileDrop.classes.add('file-drop-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
        span.classes.add('span-ondrag');
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
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
      newFileDrop.classes.add('file-drop-ondrag');
      recycle.classes.add('recycle-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
        span.classes.add('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      }
    });

    d.onDragEnd.listen((event) {
      newFileDrop.classes.remove('file-drop-ondrag');
      recycle.classes.remove('recycle-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      List<SpanElement> closedList = querySelectorAll('.list-folder');
      List<SpanElement> folderList = new List.from(spanList)..addAll(closedList);
      for (SpanElement span in folderList) {
        span.classes.remove('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE', body: 'editor-ondrag'));
      }
    });
  }

  /// Handles an Explorer add update for a single file.
  void addUpdate(String path) {
    SimpleFile sFile = new SimpleFile.fromPath(path, workspacePath, false);
    var parentPath = pathLib.dirname(sFile.path);

    // Try to detect the parent, and if it doesn't exist then create the element for it.
    LIElement li = querySelector("[data-path='$parentPath']");
    String curPath = '';

    // Iterate through the path checking to see if the folder exists
    var split = parentPath.replaceFirst(pathLib.normalize(workspacePath), '').split('/');
    for (int i = 1; i < split.length; i++) {
      curPath += split[i];
      LIElement curLi = querySelector('[data-path="${pathLib.join(workspacePath, curPath)}"]');
      if (curLi == null && pathLib.join(workspacePath, curPath) != workspacePath) {
        newElementFromFile(new SimpleFile.fromPath(pathLib.join(workspacePath, curPath), workspacePath, true)).then((result) {
        });
      }
      if (i != split.length - 1) {
        curPath += '/';
      }
    }
    newElementFromFile(sFile);
  }

  /// Handles an Explorer remove update for a single file.

  void removeUpdate(String path) {
    LIElement li = querySelector("[data-path='$path']");

    // Case to deal with removeUpdate grabbing null objects when items are renamed
    if (li == null) {
    } else {
      UListElement ul = li.parent;
      ul.children.remove(li);
    }
  }


  /// Sets up a new HTML element from a SimpleFile.
  Future newElementFromFile(SimpleFile file, [bool expanded]) {
    Completer completer = new Completer();
    completer.complete(true);

    LIElement li = generateLiHtml(file, expanded);
    String truePath = pathLib.dirname(li.dataset['path']);

    // Register double-click event handler for file renaming.
    li.children[1].onDoubleClick.listen((e) {
      renameEventHandler(li, file);
      // To prevent the rename event from propagating up the directory tree.
      e.stopPropagation();
    });

    // Set up drag and drop for file open & delete.
    dragSetup(li, file);

    UListElement dirElement;
    if (file.parentDir == '' && !file.path.contains('/.')) {

      dirElement = querySelector('#explorer-body');
      dirElement.append(li);
    } else if (!file.path.contains('/.') && !file.path.contains('CMakeLists.txt')) {
      var validPath = removeSpaces(truePath);
      var validParent = removeSpaces(file.parentDir);
      dirElement = querySelector("[data-name=explorer-ul-${validParent}][data-path='$validPath']");
      dirElement.append(li);
    }

    return completer.future;
  }

  /// First Directory List Generation

  void initialDirectoryList(String raw) {
    var files = fileList(raw);
    var pathList = [];

    UListElement explorer = querySelector('#explorer-body');
    explorer.innerHtml = '';

    for (SimpleFile file in files) {
      newElementFromFile(file, false);
      pathList.add(file.path);
    }
    cs.add(new CommanderMessage('EDITOR', 'PATH_PASS', body: pathList));
  }

  /// Redraws all file explorer views.
  void generateDirectoryList(String raw) {
    var files = fileList(raw);
    var folderStateList = {};
    for (var file in files) {
      if (file.isDirectory == true) {
        LIElement folder = querySelector('[data-path="${file.path}"]');
        if (folder != null) {
          folder.dataset['expanded'] == "true" ? folderStateList[file] = true : folderStateList[file] = false;
        }
      }
    }

    // Set the explorer list to empty for a full refresh.
    UListElement explorer = querySelector('#explorer-body');
    explorer.innerHtml = '';

    for (SimpleFile file in files) {
      file.isDirectory == true ? newElementFromFile(file, folderStateList[file]) : newElementFromFile(file);
    }
  }

  /// Update the explorer view in case of nested folder movement to cover for empty folders

  void refreshPage(String raw) {
    var files = fileList(raw);

    for (SimpleFile file in files) {
      //only check for folders
      if (file.isDirectory == true) {
        LIElement curFolder = querySelector('[data-path="${file.path}"]');
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
