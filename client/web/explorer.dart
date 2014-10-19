part of client;

class UpDroidExplorer {
  WebSocket ws;
  UpDroidEditor ed;
  String absolutePathPrefix;
  
  DivElement editorDiv;
  Dropzone dzEditor;

  ParagraphElement recycle;
  Dropzone dzRecycle;
  
  UpDroidExplorer(WebSocket ws, UpDroidEditor ed) {
    this.ws = ws;
    this.ed = ed;
    absolutePathPrefix = '';
    
    editorDiv = querySelector('#editor');
    dzEditor = new Dropzone(editorDiv);
    
    recycle = querySelector('#recycle');
    dzRecycle = new Dropzone(recycle);
    
    registerExplorerEventHandlers();
  }
  
  registerExplorerEventHandlers() {
    dzRecycle.onDragEnter.listen((e) {
      recycle.style
        ..color = '#ffffff'
        ..borderColor = '#ffffff';
    });
    
    dzRecycle.onDragLeave.listen((e) {
      recycle.style
        ..color = '#268bd2'
        ..borderColor = '#268bd2';
    });
    
    dzRecycle.onDrop.listen((e) {
      var path = e.draggableElement.dataset['path'];
      ws.send('[[EXPLORER_DELETE]]' + path);
    });
    
    // This is buggy - doesn't reliably get set when entered
    // from the left side.
    dzEditor.onDragEnter.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        editorDiv.style
          ..borderColor = '#ffffff';
      }
    });
    
    dzEditor.onDragLeave.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        editorDiv.style
          ..borderColor = '#268bd2';
      }
    });
    
    dzEditor.onDrop.listen((e) {
      var isDir = e.draggableElement.dataset['isDir'];
      if (isDir == 'false') {
        ed.openFile = e.draggableElement.dataset['path'];
        ws.send('[[EDITOR_OPEN]]' + ed.openFile);
      }
    });
  }
  
  void updateFileExplorer(String data) {
    // Set the explorer list to empty for a full refresh.
    UListElement explorer = querySelector('#explorer-top');
    explorer.innerHtml = '';
    
    // Strip the brackets/single-quotes and split by ','.
    data = data.replaceAll(new RegExp(r"(\[|\]|')"), '');
    List<String> entities = data.split(',');
    
    // Build SimpleFile list our of raw strings.
    var files = [];
    for (String entity in entities) {
      files.add(new SimpleFile(entity, absolutePathPrefix));
    }
    
    // Sorting the files results in a null object exception for some reason.
    //files.sort();

    // Refresh the FileExplorer.
    UListElement dirElement;
    for (SimpleFile file in files) {
      dirElement = (file.parentDir == 'root') ? querySelector('#explorer-top') : querySelector('#explorer-ul-${file.parentDir}');

      LIElement li = new LIElement();
      li
        ..id = file.name
        ..dataset['path'] = file.path
        ..dataset['isDir'] = file.isDirectory.toString()
        ..draggable = true
        ..classes.add('explorer-li');
      
      SpanElement span = new SpanElement();
      var glyphType = (file.isDirectory) ? 'glyphicon-folder-open' : 'glyphicon-file';
      span.classes.addAll(['glyphicon', glyphType]);
      li.children.add(span);
      
      li.appendHtml(' ${file.name}');
      
      if (file.isDirectory) {
        UListElement ul = new UListElement();
        ul
          ..id = 'explorer-ul-${file.name}'
          ..classes.addAll(['explorer', 'explorer-ul']);
        li.children.add(ul);
      }
      
      Draggable d = new Draggable(li, avatarHandler: new AvatarHandler.clone());
      
      d.onDragStart.listen((event) {
        recycle.style
          ..color = '#268bd2'
          ..borderColor = '#268bd2';
        if (!file.isDirectory) {
          editorDiv.style
            ..borderColor = '#268bd2';
        }
      });
      
      d.onDragEnd.listen((event) {
        recycle.style
          ..color = '#333333'
          ..borderColor = '#dddddd';
        if (!file.isDirectory) {
          editorDiv.style
            ..borderColor = '#dddddd';
        }
      });
      
      dirElement.children.add(li);
    }
  }
}

class SimpleFile implements Comparable {
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
  
  @override
  int compareTo(SimpleFile other) {
    return name.compareTo(other.name);
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