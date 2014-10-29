part of client;

/// [UpDroidExplorer] manages the data for the file explorer on the client
/// side and all associated views. It also facilitates file operation requests
/// to the server side.
class UpDroidExplorer {
  WebSocket ws;
  UpDroidEditor ed;
  String absolutePathPrefix;

  HRElement rootline;
  ParagraphElement recycle;
  Dropzone dzRootLine;
  Dropzone dzRecycle;
  Dropzone dzEditor;
  
  UpDroidExplorer(WebSocket ws, UpDroidEditor ed) {
    this.ws = ws;
    this.ed = ed;
    absolutePathPrefix = '';

    dzEditor = new Dropzone(ed.editorDiv);
    
    recycle = querySelector('#recycle');
    dzRecycle = new Dropzone(recycle);
    
    rootline = querySelector('#file-explorer-hr');
    dzRootLine = new Dropzone(rootline);
    
    registerExplorerEventHandlers();
  }
  
  /// Sets up the event handlers for the file explorer. Mostly mouse events.
  registerExplorerEventHandlers() {
    dzRootLine.onDragEnter.listen((e) => rootline.classes.add('file-explorer-hr-entered'));
    dzRootLine.onDragLeave.listen((e) => rootline.classes.remove('file-explorer-hr-entered'));
    
    dzRootLine.onDrop.listen((e) {
      var currentPath = e.draggableElement.dataset['path'];
      var newPath = '$absolutePathPrefix${e.draggableElement.id}';
      ws.send('[[EXPLORER_MOVE]]' + currentPath + ' ' + newPath);
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
        ed.editorDiv.classes.add('editor-entered');
      }
    });
    
    dzEditor.onDragLeave.listen((e) => ed.editorDiv.classes.remove('editor-entered'));
    
    dzEditor.onDrop.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        ed.openFile = e.draggableElement.dataset['path'];
        ws.send('[[EDITOR_OPEN]]' + ed.openFile);
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
      files.add(new SimpleFile(entity, absolutePathPrefix));
    }
    
    return files;
  }
  
  /// Returns a generated [LIElement] with inner HTML based on
  /// the [SimpleFile]'s contents.
  LIElement generateLiHtml(file) {
    LIElement li = new LIElement();
    li
      ..id = file.name
      ..dataset['path'] = file.path
      ..dataset['isDir'] = file.isDirectory.toString()
      ..draggable = true
      ..classes.add('explorer-li');
    
    SpanElement span = new SpanElement();
    
    SpanElement glyphicon = new SpanElement();
    var glyphType = (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
    glyphicon.classes.addAll(['glyphicon', glyphType]);
    dropSetup(glyphicon, file);
    span.children.add(glyphicon);

    span.appendHtml(' ${file.name}');
    
    li.children.add(span);
    
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
        var currentPath = e.draggableElement.dataset['path'];
        var newPath = '${span.parent.dataset['path']}/${e.draggableElement.id}';
        // Avoid an exception thrown when the new name already exists.
        if (currentPath != span.parent.dataset['path']) {
          ws.send('[[EXPLORER_MOVE]]' + currentPath + ' ' + newPath);
        }
      });
    }
  }
  
  /// Handles file renaming with a double-click event.
  void renameEventHandler(LIElement li, SimpleFile file) {
    if (!li.className.contains('editing')) {
      li.classes.add('editing');
      
      // Save the text in case editing is cancelled.
      var currentName = li.id;
      var currentPath = li.dataset['path'];
      
      // Create a wrapper span for the glyphicon and input to insert into li.
      SpanElement span = new SpanElement();
      
      // Recreate the glphyicon because it's hard to replace just the text portion of
      // the top level li.
      SpanElement glyphicon = new SpanElement();
      var glyphType = (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
      glyphicon.classes.addAll(['glyphicon', glyphType]);
      span.children.add(glyphicon);
      
      InputElement input = new InputElement();
      // TODO: need to make field width scale to the user's input.
      // Using a 'contenteditable' <span> instead of an <input> is a possible option.
      input.width = 100;
      input.placeholder = file.name;
      
      // TODO: Fix this - does not work for some reason.
      input.focus();
      
      input.onKeyUp.listen((e) {
        var keyEvent = new KeyEvent.wrap(e);
        if (keyEvent.keyCode == KeyCode.ENTER) {
          var newPath = currentPath.replaceFirst(currentName, input.value);
          ws.send('[[EXPLORER_RENAME]]' + currentPath + ' ' + newPath);
        } else {
          //input.size = auto;
        }
      });
      
      span.children.add(input);
      
      li.children[0] = span;
    }
  }
  
  /// Sets up a [Draggable] for the [LIElement] to handle file open and delete.
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
        ed.editorDiv.classes.add('editor-ondrag');
      }
    });
    
    d.onDragEnd.listen((event) {
      rootline.classes.remove('file-explorer-hr-ondrag');
      recycle.classes.remove('recycle-ondrag');
      List<SpanElement> spanList = querySelectorAll('.glyphicon-folder-open');
      for (SpanElement span in spanList) {
        span.classes.remove('span-ondrag');
      }
    });
  }
  
  /// Redraws all file explorer views.
  void syncExplorer(String data) {
    var files = fileList(data);
    
    // Set the explorer list to empty for a full refresh.
    UListElement explorer = querySelector('#explorer-top');
    explorer.innerHtml = '';

    // Generate the HTML for the File Explorer.
    for (SimpleFile file in files) {
      LIElement li = generateLiHtml(file);
      
      // Register double-click event handler for file renaming.
      li.onDoubleClick.listen((e) {
        renameEventHandler(li, file);
        // To prevent the rename event from propagating up the directory tree.
        e.stopPropagation();
      });
      
      // Set up drag and drop for file open & delete.
      dragSetup(li, file);
      
      UListElement dirElement = (file.parentDir == 'root') ? querySelector('#explorer-top') : querySelector('#explorer-ul-${file.parentDir}');
      dirElement.children.add(li);
    }
  }
}

/// Container class that extracts data from the raw file text passed in from
/// the server over [WebSocket]. Primarily used for generating the HTML views
/// in the file explorer that represent the filesystem.
class SimpleFile {
  String raw;
  String path;
  bool isDirectory;
  String name;
  String parentDir;
  
  SimpleFile(String raw, String prefix) {
    this.raw = raw;
    String workingString = stripFormatting(raw, prefix);
    getData(workingString);
  }
  
  String stripFormatting(String raw, String prefix) {
    raw = raw.trim();
    isDirectory = raw.startsWith('Directory: ') ? true : false;
    raw = raw.replaceFirst(new RegExp(r'(Directory: |File: )'), '');
    
    // Save the absolute path.
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
      parentDir = 'root';
    }
  }
}