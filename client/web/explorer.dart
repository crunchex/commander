part of client;

class FileExplorer {
  WebSocket ws;
  String directoryPath;
  
  FileExplorer(WebSocket ws) {
    this.ws = ws;
    directoryPath = '';
  }
  
  void updateFileExplorer(String data) {
    // Strip the packet header and brackets and split by ','
    String strippedHeader = data.replaceAll(new RegExp(r'(Directory:|File:|\s)'), '');
    strippedHeader = strippedHeader.replaceAll(new RegExp(r'(\[|\])'), '');
    strippedHeader = strippedHeader.replaceAll(r"'", '');
    List<String> files = strippedHeader.split(',');
    
    UListElement explorer = querySelector('#ul-explorer');
    explorer.innerHtml = '';
    
    for (String file in files) {
      file = file.replaceFirst(directoryPath, '');
      explorer.appendHtml('<li><button type="button" class="btn btn-xs">${file}</button></li>');
    }
  }
}