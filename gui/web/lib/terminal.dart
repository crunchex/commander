library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'console_helper.dart';

const bool debug = false;

/// A class for keeping track of inputHandling state.
/// Stand-in for what should be an enum.
class InputMode {
  static int dump = 0;
  static int normal = 1;
  static int escape = 2;
}

/// A class for rendering a terminal emulator in a [DivElement] (param).
/// [stdout] needs to receive individual UTF8 integers and will handle
/// them appropriately.
class Terminal {
  DivElement div;
  int _charWidth, _charHeight;
  List _cursorXY;
  
  // This should really be an enum.
  int _inputSwitch;
  
  List<int> _escapeCode;
  List<int> _outString;
  
  StreamController stdout;
  StreamController _escapeCodes;
  StreamController _normalStrings;
  
  Terminal (DivElement div) {
    _inputSwitch = InputMode.normal;
    _charWidth = 10;
    _charHeight = 17;
    _cursorXY = [0, 0];

    this.div = div;
    _escapeCode = [];
    _outString = [];

    stdout = new StreamController<String>();
    _escapeCodes = new StreamController<String>();
    _normalStrings = new StreamController<String>();
    
    registerEventHandlers();
    initDisplay();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth - 1;
  int get _rows => div.borderEdge.height ~/ _charHeight - 1;
  
  void registerEventHandlers() {
    stdout.stream.listen((out) {
      if (debug) {
        print(out);
        return;
      }
      
      if (out == '27' || _inputSwitch == InputMode.escape) {
        _escapeCodes.add(out);
        
      }
    });
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
    switch (output) {
      case '27':
        // Escape Sequence detected.
        _escapeCode.add(int.parse(output));
        break;
      case '10':
        // New Line detected.
        
    }
    
    // Do something with the string before resetting it.
    String str = UTF8.decode(_outString);
    print('str ' + str);
    drawString(str);
    _outString = [];
  }
  
  void drawString(String str) {
    print('cursor x before: ' + _cursorXY[0]);
    SpanElement strSpan = new SpanElement();
    strSpan.innerHtml = str;
    div.children[_cursorXY[0]].children.insert(0, strSpan);
    _cursorXY[0]++;
    print('cursor x after: ' + _cursorXY[0]);
  }
}