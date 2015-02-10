library terminal;

import 'dart:html';
import 'dart:async';

class Terminal {
  DivElement div;
  int _charWidth, _charHeight;
  List _cursorXY;
  
  StreamController stdout;
  
  Terminal (this.div) {
    _charWidth = 10;
    _charHeight = 13;

    stdout = new StreamController<List<int>>();
    
    registerEventHandlers();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth;
  int get _rows => div.borderEdge.height ~/ _charHeight;
  
  void registerEventHandlers() {
    stdout.stream.listen((data) => print(data));
  }
}