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
      // Escape detected.
      if (out == '27') {
        _inputSwitch = InputMode.escape;
        return;
      }
      
      if (_inputSwitch == InputMode.escape) {
        _escapeCodes.add(out);
      } else if (_inputSwitch == InputMode.normal) {
        _normalStrings.add(out);
      }

    });
    
    _escapeCodes.stream.listen((singleCode) {
      _escapeCode.add(int.parse(singleCode));
      String escapeString = UTF8.decode(_escapeCode);

      switch (escapeString) {
        case '[0m':
          print('reset all: ' + escapeString);
          _inputSwitch = InputMode.normal;
          _escapeCode = [];
          break;
      }
    });
    
    _normalStrings.stream.listen((singleCode) {
      if (singleCode == '10' || singleCode == '13') {
        drawString(UTF8.decode(_outString));
        _outString = [];
        return;
      }
      
      _outString.add(int.parse(singleCode));
    });
  }
  
  initDisplay() {
    for (var i = 0; i < _rows; i++) {
      DivElement row = new DivElement();
      row.classes.add('termrow');

      for (var j = 0; j < _cols; j++) {
        row.innerHtml += "&nbsp";
      }
      
      div.children.add(row);
    }
  }
  
  void drawString(String str) {
    // Copy away the current text and remove.
    DivElement row = div.children[_cursorXY[1]];
    String tmp = row.innerHtml;
    row.innerHtml = "";

    SpanElement strSpan = new SpanElement();
    strSpan.text = str;
    
    // Append the new span and reinsert the original text.
    row.append(strSpan);
    row.appendHtml(tmp);
    
    // Move the cursor one line down (Y).
    _cursorXY[1]++;
  }
}