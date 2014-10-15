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
    
    DivElement well = querySelector('#well-explorer');
    well.innerHtml = '<p>File Explorer</p><ul>';
    
    for (String file in files) {
      file = file.replaceFirst(directoryPath, '');
      well.appendHtml('<li>/${file}</li>');
    }
    well.appendHtml('</ul>');
  }

}