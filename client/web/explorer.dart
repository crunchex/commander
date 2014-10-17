part of client;

class FileExplorer {
  Element _dragSourceEl;
  WebSocket ws;
  String absolutePathPrefix;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    absolutePathPrefix = '';
    
    registerExplorerEventHandlers();
  }
  
  void registerExplorerEventHandlers() {
    var elements = document.querySelectorAll('#explorer-top .btn, #recycle');
    for (Element element in elements) {
      element.onDragStart.listen(_onDragStart);
      element.onDragEnd.listen(_onDragEnd);
      element.onDragEnter.listen(_onDragEnter);
      element.onDragOver.listen(_onDragOver);
      element.onDragLeave.listen(_onDragLeave);
      element.onDrop.listen(_onDrop);
    }
  }

  void _onDragStart(MouseEvent event) {
    Element dragTarget = event.target;
    dragTarget.classes.add('moving');
    _dragSourceEl = dragTarget;
    event.dataTransfer.effectAllowed = 'move';
    event.dataTransfer.setData('text/html', dragTarget.innerHtml);
    
    
    print('dragging');
  }

  void _onDragEnd(MouseEvent event) {
    Element dragTarget = event.target;
    dragTarget.classes.remove('moving');
    dragTarget.classes.remove('active');
    print('end');
  }

  void _onDragEnter(MouseEvent event) {
    Element dropTarget = event.target;
    dropTarget.classes.add('over');
    print('Entering: ${dropTarget.className}');
    
    if (dropTarget.className.contains('recycle')) {
      dropTarget.style.color = '#268bd2';
      dropTarget.style.borderColor = '#268bd2';
    }
  }

  void _onDragOver(MouseEvent event) {
    // This is necessary to allow us to drop.
    event.preventDefault();
    event.dataTransfer.dropEffect = 'move';
  }

  void _onDragLeave(MouseEvent event) {
    Element dropTarget = event.target;
    dropTarget.classes.remove('over');
    
    if (dropTarget.className.contains('recycle')) {
      dropTarget.style.color = '#333333';
      dropTarget.style.borderColor = '#dddddd';
    }
  }

  void _onDrop(MouseEvent event) {
    // Stop the browser from redirecting.
    event.stopPropagation();

    // Don't do anything if dropping onto the same column we're dragging.
    Element dropTarget = event.target;
    if (dropTarget.className.contains('recycle')) {
      // Set the source column's HTML to the HTML of the column we dropped on.
      dropTarget.style.color = '#333333';
      dropTarget.style.borderColor = '#dddddd';
      print('Delete this file');
    }
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
      dirElement = (file.parentDir == 'root') ? querySelector('#explorer-top') : querySelector('#explorer-${file.parentDir}');
      
      String newHtml;
      if (file.isDirectory) {
        newHtml = '<li class="explorer-li"><button type="button" draggable="true" class="btn btn-xs"><span class="glyphicon glyphicon-folder-open"></span>  ${file.name}</button><ul id="explorer-${file.name}" class="explorer explorer-ul"></ul></li>';
      } else {
        newHtml = '<li class="explorer-li"><button type="button" draggable="true" class="btn btn-xs"><span class="glyphicon glyphicon-file"></span> ${file.name}</button></li>';
      }
      dirElement.appendHtml(newHtml);
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