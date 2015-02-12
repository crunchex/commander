library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'console_helper.dart';

const bool debug = false;

/// A class for rendering a terminal emulator in a [DivElement] (param).
/// [stdout] needs to receive individual UTF8 integers and will handle
/// them appropriately.
class Terminal {
  DivElement div;
  int _charWidth, _charHeight;
  List _cursorXY;
  
  List _escapeCode;
  List _outString;
  
  StreamController stdout;
  
  Terminal (DivElement div) {
    _charWidth = 10;
    _charHeight = 17;
    _cursorXY = [0, 0];

    this.div = div;
    _escapeCode = [];
    _outString = [];

    stdout = new StreamController<List<int>>();
    
    registerEventHandlers();
    initDisplay();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth - 1;
  int get _rows => div.borderEdge.height ~/ _charHeight - 1;
  
  void registerEventHandlers() {
    stdout.stream.listen((output) => (debug) ? print(output) : handleOutput(output));
  }
  
  initDisplay() {
    print('rows ' + _rows.toString());
    for (var i = 0; i < _rows; i++) {
      DivElement row = new DivElement();
      row.classes.add('termrow');

      for (var j = 0; j < _cols; j++) {
        row.innerHtml += "&nbsp";
      }
      
      div.children.add(row);
    }
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
  
  void drawString(String str) {
    
  }
}