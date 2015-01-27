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
  HRElement rootline;
  DivElement rootlineContainer;
  ParagraphElement recycle;
  
  Draggable dragNewFile;
  Draggable dragNewFolder;

  Dropzone dzRootLineContainer;
  Dropzone dzRecycle;
  Dropzone dzEditor;
  
  UpDroidExplorer(WebSocket ws, StreamController<CommanderMessage> cs) {
    this.ws = ws;
    this.cs = cs;
    
    newFile = querySelector('#file');
    newFileDragSetup();
    
    newFolder = querySelector('#folder');
    newFolderDragSetup();

    rootline = querySelector('#file-explorer-hr');
    rootlineContainer = querySelector('#file-explorer-hr-container');
    dzRootLineContainer = new Dropzone(rootlineContainer);
    
    recycle = querySelector('#recycle');
    dzRecycle = new Dropzone(recycle);
    
    editorDiv = querySelector('#editor');
    dzEditor = new Dropzone(editorDiv);
    
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
    cs.stream
        .where((m) => m.dest == 'EXPLORER' || m.dest == 'ALL')
        .listen((m) => processMessage(m));
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_PATH')
        .listen((um) {
          workspacePath = um.body;
          ws.send('[[EXPLORER_DIRECTORY_LIST]]');
        });

    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_DIRECTORY_LIST')
        .listen((um) => generateDirectoryList(um.body));

    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_ADD')
        .listen((um) => addUpdate(um.body));
    
    ws.onMessage.transform(updroidTransformer)
        .where((um) => um.header == 'EXPLORER_REMOVE')
        .listen((um) => removeUpdate(um.body));
    
    dzRootLineContainer.onDragEnter.listen((e) => rootline.classes.add('file-explorer-hr-entered'));
    dzRootLineContainer.onDragLeave.listen((e) => rootline.classes.remove('file-explorer-hr-entered'));
    
    dzRootLineContainer.onDrop.listen((e) {
      if (e.draggableElement.className.contains('explorer-li')) {
        // The draggable is an existing file/folder.
        var currentPath = e.draggableElement.dataset['path'];
        var newPath = '$workspacePath/${e.draggableElement.dataset['trueName']}';
        ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
      } else if (e.draggableElement.id == 'file'){
        ws.send('[[EXPLORER_NEW_FILE]]' + workspacePath);   
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
  
  // Helper function removs spaces from path and file name
  
  String removeSpaces(String raw){
    var result = '';
    List<String> split = raw.split(' ');
    for(var i=0; i< (split.length); i++){
      result += split[i];
    }
    return result;
  }
  
  
  /// Returns a generated [LIElement] with inner HTML based on
  /// the [SimpleFile]'s contents.
  LIElement generateLiHtml(file) {
    LIElement li = new LIElement();
    li
      ..dataset['name'] = removeSpaces(file.name)
      ..dataset['trueName'] = (file.name)
      ..dataset['path'] = file.path
      ..dataset['isDir'] = file.isDirectory.toString()
      ..dataset['hiding'] = 'false'
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
        ..dataset['name'] = 'explorer-ul-${removeSpaces(file.name)}'
        ..dataset['path'] = removeSpaces(file.path)
        ..classes.addAll(['explorer', 'explorer-ul']);
      li.children.add(ul);
      
      glyphicon.onDoubleClick.listen((e) {
        if (glyphicon.dataset['hiding'] == 'false') {
          ul.classes.add('explorer-hidden');
          glyphicon.dataset['hiding'] = 'true';
        } else {
          ul.classes.remove('explorer-hidden');
          glyphicon.dataset['hiding'] = 'false';
        }
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
          // Avoid an exception thrown when the new name already exists.
          if (currentPath != span.parent.dataset['path']) {
            ws.send('[[EXPLORER_MOVE]]' + currentPath + ':divider:' + newPath);
          }
        } else if (e.draggableElement.id == 'file') {
          ws.send('[[EXPLORER_NEW_FILE]]' + span.parent.dataset['path']);   
        } else {
          ws.send('[[EXPLORER_NEW_FOLDER]]' + span.parent.dataset['path'] + '/untitled');
        }
      });
    }
  }
  
  /// Helper function grabs correct parent directory path
  
  String filePathGrab(var file){
    String result = '';
    String raw = file.path;
    List split = raw.split('/');
    for(var i=0; i<(split.length -1); i++){
      result += split[i];
      result += "/";
    }
    return result;
  }
  
  /// Handles file renaming with a double-click event.
  void renameEventHandler(LIElement li, SimpleFile file) {
    if (!li.className.contains('editing')) {
      li.classes.add('editing');
      
      InputElement input = new InputElement();
      input.placeholder = '';
      
      // TODO: Fix this - does not work for some reason.
      input.focus();
      
      input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          var newPath = filePathGrab(file) + input.value;
          
          ws.send('[[EXPLORER_RENAME]]' + file.path + ':divider:' + newPath);
          
          // Remove this element once editing is complete, as the new one will soon appear.
          if(file.path != newPath){
            UListElement ul = li.parent;
                      ul.children.remove(li);
          }
          if(file.path == newPath){
            li.classes.remove('editing');
            li.children.remove(input);
            SpanElement filename = new SpanElement();
                filename
                    ..classes.add('filename')
                    ..text = file.name;
                li.children.add(filename);
          }
          
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
  void addUpdate(String path) {
    SimpleFile sFile = new SimpleFile.fromPath(path, workspacePath, false);
    var parentPath = filePathGrab(sFile);
    print(parentPath);
    
    // Try to detect the parent, and if it doesn't exist then create the element for it.
    LIElement li = querySelector("[data-path='$parentPath']");
    if (li == null) {
      print('parent not found at: $parentPath');
      //new SimpleFile.fromPath(parentPath, workspacePath, true);
      newElementFromFile(new SimpleFile.fromPath(parentPath, workspacePath, true)).then((result) {
        //newElementFromFile(sFile);
      });
    }
  }
  
  // fixpoint
  
  /// Handles an Explorer remove update for a single file.
  
  void removeUpdate(String path) {
    LIElement li = querySelector("[data-path='$path']");
    
    // Case to deal with removeUpdate grabbing null objects when items are renamed
    if(li == null){  
    }
    
    else{
    UListElement ul = li.parent;
    ul.children.remove(li);
    }
  }
  
  /// Helper function grabs correct parent directory path
  
  String pathGrab(LIElement li){
    String result = '';
    String raw = li.dataset['path'];
    List split = raw.split('/');
    for(var i=0; i<(split.length -1); i++){
      result += split[i];
      if(i != split.length -2){
        result += "/";
      }
    }
    return result;
  }
  
  
  /// Sets up a new HTML element from a SimpleFile.
  Future newElementFromFile(SimpleFile file) {
    Completer completer = new Completer();
    completer.complete(true);

    LIElement li = generateLiHtml(file);
    String truePath = pathGrab(li);

    // Register double-click event handler for file renaming.
    li.children[1].onDoubleClick.listen((e) {
      renameEventHandler(li, file);
      // To prevent the rename event from propagating up the directory tree.
      e.stopPropagation();
    });
    
    // Set up drag and drop for file open & delete.
    dragSetup(li, file);
    
    UListElement dirElement;
    if(file.parentDir == ''){
      dirElement = querySelector('#explorer-top');
      dirElement.children.add(li);
    }
    else{
      var validPath = removeSpaces(truePath);
      var validParent = removeSpaces(file.parentDir);
      dirElement = querySelector("[data-name=explorer-ul-${validParent}][data-path='$validPath']");
        dirElement.children.add(li);
    }
    
    return completer.future;
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
  
  SimpleFile.fromPath(String raw, String prefix, bool isDir) {
    
    // What is this for??
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