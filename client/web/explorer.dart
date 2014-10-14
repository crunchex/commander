part of client;

class FileExplorer {
  WebSocket ws;
  ButtonElement refresh;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    this.refresh = querySelector('#button-refresh');
    
    registerExplorerEventHandlers();
  }

  void registerExplorerEventHandlers() {
    refresh.onClick.listen((e) {
      ws.send('REQUEST FILESYSTEM UPDATES');
    });
  }
  
  void updateFileExplorer(String data) {
    String strippedHeader = data.replaceFirst(new RegExp('DIRECTORY_LIST: '), '');
    List<String> files = strippedHeader.split(' ');
    
    DivElement well = querySelector('#well-explorer');
    well.innerHtml = '<p>File Explorer</p><ul>';
    
    for (String file in files) {
      well.appendHtml('<li>${file}</li>');
    }
    well.appendHtml('</ul>');
  }

}