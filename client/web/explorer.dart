part of client;

class FileExplorer {
  WebSocket ws;
  String directoryPath;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    directoryPath = '';
  }
  
  void updateFileExplorer(String data) {
    // Set the explorer list to empty for a full refresh
    UListElement explorer = querySelector('#explorer-top');
    explorer.innerHtml = '';
    
    // Strip the brackets/single-quotes and split by ','
    data = data.replaceAll(new RegExp(r'(Directory: |File: )'), '');
    data = data.replaceAll(new RegExp(r"(\[|\]|')"), '');
    List<String> entities = data.split(',');
    
    // Move all the directories to the front
    entities.sort();
    
    List<String> dirString = ['#explorer-top'];
    for (String entity in entities) {
      // Strip the absolute path and trim if necessary
      entity = entity.replaceFirst(directoryPath, '');
      entity = entity.trimLeft();

      if (entity.contains('.')) {
        UListElement dirElement = querySelector(dirString[dirString.length - 1]);
        dirElement.appendHtml('<li class="explorer"><button type="button" class="btn btn-xs">${entity}</button></li>');
        dirString = ['#explorer-top'];
      } else {
        String newHtml = '<li class="explorer"><button type="button" class="btn btn-xs">' + entity + '</button><ul id="explorer-' + entity.replaceAll('/', '-') + '" class="explorer"></ul></li>';
        UListElement dirElement = querySelector(dirString[dirString.length - 1]);
        dirElement.appendHtml(newHtml);
        dirString.add('#explorer-' + entity.replaceAll('/', '-'));
      }
    }
  }
}