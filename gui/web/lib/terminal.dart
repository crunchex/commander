library terminal;

import 'dart:html';
import 'dart:async';

class Terminal {
  DivElement div;
  Stream stdin, stdout;
  int charWidth, charHeight;
  List cursorXY;
  
  Terminal (this.div, this.stdin, this.stdout) {
    charWidth = 10;
    charHeight = 13;
  }
  
  int get cols => div.borderEdge.width ~/ charWidth;
  int get rows => div.borderEdge.height ~/ charHeight;
}