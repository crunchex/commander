library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

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
  int _charWidth, _charHeight, bufferIndex;
  List _cursorXY;
  
  // This should really be an enum.
  int _inputSwitch;
  
  List<int> _escapeCode;
  List<int> _outString;
  List<SpanElement> _buffer;
  
  StreamController stdout;
  StreamController _escapeCodes;
  StreamController _normalStrings;
  
  static const int ESC = 27;
  
  Terminal (DivElement div) {
    _inputSwitch = InputMode.normal;
    _charWidth = 10;
    _charHeight = 17;
    _cursorXY = [0, 0];
    bufferIndex = 0;

    this.div = div;
    _escapeCode = [];
    _outString = [];
    _buffer = [];

    stdout = new StreamController<String>();
    _escapeCodes = new StreamController<int>();
    _normalStrings = new StreamController<int>();
    
    _registerEventHandlers();
    _initDisplay();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth - 1;
  int get _rows => div.borderEdge.height ~/ _charHeight - 1;
  
  bool get atTop => bufferIndex <= 0;
  bool get atBottom => bufferIndex >= _buffer.length - _rows;
  
  void _registerEventHandlers() {
    stdout.stream.listen((out) {
      int outInt = int.parse(out);

      // Escape detected.
      if (outInt == ESC) {
        _escapeCodes.add(outInt);
        _inputSwitch = InputMode.escape;
        return;
      }
      
      if (_inputSwitch == InputMode.escape) {
        _escapeCodes.add(outInt);
      } else if (_inputSwitch == InputMode.normal) {
        _normalStrings.add(outInt);
      }
    });
    
    _escapeCodes.stream.listen((singleCode) {
      _escapeCode.add(singleCode);

      switch (singleCode) {
        case 109: // <ESC>[{attr1};...;{attrn}m
          _setAttributeMode(_escapeCode);
          _escapeCode = [];
          print('switching to normal mode!');
          _inputSwitch = InputMode.normal;
          break;    
      }
    });
    
    _normalStrings.stream.listen((singleCode) {
      _outString.add(singleCode);
      
      if (!(singleCode == 10 || singleCode == 13)) {
        print('appending to normal string');
        return;
      }

      // Add the new string to buffer.
      SpanElement newSpan = new SpanElement();
      newSpan.text = UTF8.decode(_outString);
      _buffer.add(newSpan);
      _outString = [];
      
      if (!atBottom) {
        bufferIndex++;
      }
      drawDisplay();
    });
  }
  
  void _setAttributeMode(List<int> mode) {
    print('setting attribute mode! ' + mode.toString());
  }
  
  void _initDisplay() {
    for (var i = 0; i < _rows; i++) {
      DivElement row = new DivElement();
      row.classes.add('termrow');

      for (var j = 0; j < _cols; j++) {
        row.innerHtml += "&nbsp";
      }
      
      div.children.add(row);
    }
  }
  
  /// Returns a long string of &nbsp, one per Terminal col.
  String _generateNbsp() {
    String nbsp = '';
    for (var i = 0; i < _cols; i++) {
      nbsp = nbsp + '&nbsp;';
    }

    return nbsp;
  }
  
  /// Updates the display in canonical mode based
  /// on contents of the buffer.
  void drawDisplay() {
    for (int i = 0; i < div.children.length; i++) {
      DivElement row = div.children[i];
      
      // Nothing in the buffer at this row, skip rest.
      if (i >= _buffer.length) continue;
      
      // Reset the row.
      row.innerHtml = "";
      
      // Start with the standard long string of &nbsp, then trim
      // to fit the SpanElement.
      String nbsp = _generateNbsp();
      print('i: ' + i.toString());
      print('buffer len: ' + _buffer.length.toString());
      nbsp = nbsp.substring(_buffer[bufferIndex + i].text.length * 6);
      
      // Append the span from buffer and reinsert the original text after the span.
      row.append(_buffer[bufferIndex + i]);
      row.appendHtml(nbsp);
    }
  }
}