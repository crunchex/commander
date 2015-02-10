library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'console_helper.dart';

class Terminal {
  DivElement div;
  int _charWidth, _charHeight;
  List _cursorXY;
  
  List _escapeCode;
  List _outString;
  
  StreamController stdout;
  
  Terminal (this.div) {
    _charWidth = 10;
    _charHeight = 13;
    
    _escapeCode = [];
    _outString = [];

    stdout = new StreamController<List<int>>();
    
    registerEventHandlers();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth;
  int get _rows => div.borderEdge.height ~/ _charHeight;
  
  void registerEventHandlers() {
    stdout.stream.listen((output) => handleOutput(output));
  }
  
  /// Takes the raw stdout from shell per individual UTF8 int (as a string)
  /// and and handles it appropriately.
  void handleOutput(String output) {
    // Append if NL or start of Escape Sequence is undetected.
    if (output != '10' && output != '27') {
      _outString.add(int.parse(output));
      return;
    }
    
    // Do something with the string before resetting it.
    print(UTF8.decode(_outString));
    _outString = [];
  }
}