part of client;

class FileExplorer {
  WebSocket ws;
  String absolutePathPrefix;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    absolutePathPrefix = '';
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
        newHtml = '<li class="explorer"><button type="button" class="btn btn-xs">' + file.name + '</button><ul id="explorer-' + file.name + '" class="explorer"></ul></li>';
      } else {
        newHtml = '<li class="explorer"><button type="button" class="btn btn-xs">${file.name}</button></li>';
      }
      dirElement.appendHtml(newHtml);
    }
  }
}

class SimpleFile implements Comparable {
  bool isDirectory;
  String name;
  String parentDir;
  
  SimpleFile(String raw, String prefix) {
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