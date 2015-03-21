part of updroid_server;

class CmdrEditor {
  static const String guiName = 'UpDroidEditor';

  int editorNum = 1;
  WebSocket _ws;

  CmdrEditor(WebSocket ws) {
    _ws = ws;
  }
}