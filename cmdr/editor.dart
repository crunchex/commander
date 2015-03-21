part of updroid_server;

class CmdrEditor {
  int editorNum = 1;
  WebSocket _ws;

  CmdrEditor(WebSocket ws) {
    _ws = ws;
  }
}