part of client;

class FileExplorer {
  Element _dragSourceEl;
  WebSocket ws;
  String absolutePathPrefix;

  Dropzone dzRecycle;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    absolutePathPrefix = '';
    dzRecycle = new Dropzone(querySelector('#recycle'));
  }

  
  void updateFileExplorer(String data) {
    // Set the explorer list to empty for a full refresh
    UListElement explorer = querySelector('#explorer-top');
    explorer.innerHtml = '';
    
    // Strip the brackets/single-quotes and split by ','
    data = data.replaceAll(new RegExp(r"(\[|\]|')"), '');
    List<String> entities = data.split(',');
    
    // Build SimpleFile list our of raw strings
    var files = [];
    for (String entity in entities) {
      files.add(new SimpleFile(entity, absolutePathPrefix));
    }
    
    // Sorting the files results in a null object exception for some reason
    //files.sort();

    // Refresh FileExplorer
    UListElement dirElement;
    for (SimpleFile file in files) {
      dirElement = (file.parentDir == 'root') ? querySelector('#explorer-top') : querySelector('#explorer-ul-${file.parentDir}');

      LIElement li = new LIElement();
      li
        ..id = file.name
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
       
      dirElement.children.add(li);
      
      /*Draggable d = new Draggable(querySelector('#explorer-${file.name}'), avatarHandler: new AvatarHandler.clone());
      d.onDragStart.listen((event) {
        print('Drag start! ${event.draggableElement.className}');
      });*/
    }
  }
}

class SimpleFile implements Comparable {
  String raw;
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