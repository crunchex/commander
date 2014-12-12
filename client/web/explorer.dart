part of client;

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
  HRElement rootline;
  ParagraphElement recycle;
  
  Draggable dragNewFile;
  Draggable dragNewFolder;
  
  Dropzone dzRootLine;
  Dropzone dzRecycle;
  Dropzone dzEditor;
  
  UpDroidExplorer(WebSocket ws, StreamController<CommanderMessage> cs, String workspacePath) {
    this.ws = ws;
    this.cs = cs;
    this.workspacePath = workspacePath;
    
    newFile = querySelector('#file');
    newFileDragSetup();
    
    newFolder = querySelector('#folder');
    newFolderDragSetup();
    
    rootline = querySelector('#file-explorer-hr');
    dzRootLine = new Dropzone(rootline);
    
    recycle = querySelector('#recycle');
    dzRecycle = new Dropzone(recycle);
    
    editorDiv = querySelector('#editor-1');
    dzEditor = new Dropzone(editorDiv);
    
    registerExplorerEventHandlers();
    
    // Let the server know Explorer is up and ready to receive
    // the directory list.
    ws.send('[[EXPLORER_DIRECTORY_LIST]]');
  }
  
  /// Sets up the event handlers for the file explorer. Mostly mouse events.
  registerExplorerEventHandlers() {
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_LIST')
        .listen((um) => generateDirectoryList(um.body));

    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_ADD')
        .listen((um) => addUpdate(um.body));
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_REMOVE')
        .listen((um) => removeUpdate(um.body));
    
    dzRootLine.onDragEnter.listen((e) => rootline.classes.add('file-explorer-hr-entered'));
    dzRootLine.onDragLeave.listen((e) => rootline.classes.remove('file-explorer-hr-entered'));
    
    dzRootLine.onDrop.listen((e) {
      if (e.draggableElement.className.contains('explorer-li')) {
        // The draggable is an existing file/folder.
        var currentPath = e.draggableElement.dataset['path'];
        var newPath = '$workspacePath${e.draggableElement.id}';
        ws.send('[[EXPLORER_MOVE]]' + currentPath + ' ' + newPath);
      } else if (e.draggableElement.id == 'file'){
        ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath + '/untitled.cc');
      } else {
        ws.send('[[EXPLORER_NEW_FOLDER]]' + workspacePath + '/untitled');
      }
    });
    
    dzRecycle.onDragEnter.listen((e) => recycle.classes.add('recycle-entered'));
    dzRecycle.onDragLeave.listen((e) => recycle.classes.remove('recycle-entered'));
    
    dzRecycle.onDrop.listen((e) {
      var path = e.draggableElement.dataset['path'];
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
  LIElement generateLiHtml(file) {
    LIElement li = new LIElement();
    li
      ..dataset['name'] = file.name
      ..dataset['path'] = file.path
      ..dataset['isDir'] = file.isDirectory.toString()
      ..draggable = true
      ..classes.add('explorer-li');
    
    // Create a span element for the glyphicon
    SpanElement glyphicon = new SpanElement();
    var glyphType = (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
    glyphicon.classes.addAll(['glyphicon', glyphType]);
    dropSetup(glyphicon, file);
    li.children.add(glyphicon);

    // Hold the text inline with the glyphicon
    SpanElement filename = new SpanElement();
    filename
        ..classes.add('filename')
        ..text = file.name;
    li.children.add(filename);
    
    if (file.isDirectory) {
      UListElement ul = new UListElement();
      ul
        ..id = 'explorer-ul-${file.name}'
        ..classes.addAll(['explorer', 'explorer-ul']);
      li.children.add(ul);
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
          var newPath = '${span.parent.dataset['path']}/${e.draggableElement.id}';
          // Avoid an exception thrown when the new name already exists.
          if (currentPath != span.parent.dataset['path']) {
            ws.send('[[EXPLORER_MOVE]]' + currentPath + ' ' + newPath);
          }
        } else if (e.draggableElement.id == 'file') {
          ws.send('[[EXPLORER_NEW_FILE]]' + span.parent.dataset['path'] + '/untitled.cc');
        } else {
          ws.send('[[EXPLORER_NEW_FOLDER]]' + span.parent.dataset['path'] + '/untitled');
        }
      });
    }
  }
  
  /// Handles file renaming with a double-click event.
  void renameEventHandler(LIElement li, SimpleFile file) {
    if (!li.className.contains('editing')) {
      li.classes.add('editing');
      
      InputElement input = new InputElement();
      input.placeholder = file.name;
      
      // TODO: Fix this - does not work for some reason.
      input.focus();
      
      input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          var newPath = file.path.replaceFirst(file.name, input.value);
          ws.send('[[EXPLORER_RENAME]]' + file.path + ' ' + newPath);
          
          // Remove this element once editing is complete, as the new one will soon appear.
          UListElement ul = li.parent;
          ul.children.remove(li);
        } else {
          // TODO: need to make field width scale to the user's input.
          // Using a 'contenteditable' <span> instead of an <input> is a possible option.
          //input.size = auto;
        }
      });
      
      // Replace child 1 (span filename) with input.
      li.children[1] = input;
    }
  }
  
  /// Sets up a [Draggable] for the [newFile] to handle the drag event.
  void newFileDragSetup() {
    // Create a new draggable using the current element as
    // the visual element (avatar) being dragged.
    Draggable d = new Draggable(newFile, avatarHandler: new AvatarHandler.clone());
    
    // Highlight valid dropzones: rootline, editor, any workspace folder.
    d.onDragStart.listen((event) {
      rootline.classes.add('file-explorer-hr-ondrag');
      cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }   
    });
    
    d.onDragEnd.listen((event) {
      rootline.classes.remove('file-explorer-hr-ondrag');
      cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE', body: 'editor-ondrag'));
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
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
      rootline.classes.add('file-explorer-hr-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }   
    });
    
    d.onDragEnd.listen((event) {
      rootline.classes.remove('file-explorer-hr-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
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
      rootline.classes.add('file-explorer-hr-ondrag');
      recycle.classes.add('recycle-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
        span.classes.add('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_ADD', body: 'editor-ondrag'));
      }
    });
    
    d.onDragEnd.listen((event) {
      rootline.classes.remove('file-explorer-hr-ondrag');
      recycle.classes.remove('recycle-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
      if (!file.isDirectory) {
        cs.add(new CommanderMessage('EDITOR', 'CLASS_REMOVE', body: 'editor-ondrag'));
      }
    });
  }
  
  /// Handles an Explorer add update for a single file.
  void addUpdate(String path) => newElementFromFile(new SimpleFile.fromPath(path, workspacePath));
  
  /// Handles an Explorer remove update for a single file.
  void removeUpdate(String path) {
    LIElement li = querySelector("[data-path='$path']");
    UListElement ul = li.parent;
    ul.children.remove(li);
  }
  
  /// Sets up a new HTML element from a SimpleFile.
  newElementFromFile(SimpleFile file) {
    LIElement li = generateLiHtml(file);
    
    // Register double-click event handler for file renaming.
    li.onDoubleClick.listen((e) {
      renameEventHandler(li, file);
      // To prevent the rename event from propagating up the directory tree.
      e.stopPropagation();
    });
    
    // Set up drag and drop for file open & delete.
    dragSetup(li, file);
    
    UListElement dirElement = (file.parentDir == '') ? querySelector('#explorer-top') : querySelector('#explorer-ul-${file.parentDir}');
    dirElement.children.add(li);
  }
  
  /// Redraws all file explorer views.
  void generateDirectoryList(String raw) { 
    var files = fileList(raw);
    
    // Set the explorer list to empty for a full refresh.
    UListElement explorer = querySelector('#explorer-top');
    explorer.innerHtml = '';

    for (SimpleFile file in files) {
      newElementFromFile(file);
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
  
  SimpleFile.fromPath(String raw, String prefix) {
    path = raw;
    isDirectory = false;
    getData(raw.replaceFirst(prefix, ''));
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