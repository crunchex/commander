library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';

part 'model.dart';

const bool debug = false;

/// A class for keeping track of inputHandling state.
/// Stand-in for what should be an enum.
class InputMode {
  static int normal = 1;
  static int escape = 2;
}

/// A class for rendering a terminal emulator in a [DivElement] (param).
/// [stdout] needs to receive individual UTF8 integers and will handle
/// them appropriately.
class Terminal {
  // Public
  DivElement div;
  StreamController stdout;

  // Private
  int _charWidth, _charHeight, _inputSwitch;
  List<int> _cursorXY;
  List<SpanElement> _buffer;
  Model _model;
  
  static const int ESC = 27;
  
  Terminal (this.div) {
    stdout = new StreamController<String>();

    _charWidth = 11;
    _charHeight = 14;
    _inputSwitch = InputMode.normal;
    _model = new Model(_rows, _cols);
    
    _registerEventHandlers();
    _initDisplay();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth - 1;
  int get _rows => div.borderEdge.height ~/ _charHeight - 1;
  
  void _registerEventHandlers() {
    stdout.stream.listen((String out) {
      List<int> output = JSON.decode(out);

      List<int> outString = [];
      List<int> escapeSequence = [];
      for (int i = 0; i < output.length; i++) {
        int code = output[i];
        if (code == ESC) _inputSwitch = InputMode.escape;
 
        // Append code
        if (_inputSwitch == InputMode.escape) {
          escapeSequence.add(code);
        } else if (_inputSwitch == InputMode.normal) {
          outString.add(code);
        }
        
        // Let this be the last loop if we're at the end of the message
        // or the end of an escape sequence is detected.
        if (_inputSwitch == InputMode.escape) {
          if (_detectEscapeEnd(code)) {
            _setAttributeMode(escapeSequence);
            escapeSequence = [];
          }
        } else if (_inputSwitch == InputMode.normal) {
          if (i == output.length - 1 || code == 10) {
            _handleOutString(outString);
            outString = [];
          }
        }
      }
    });
  }
  
  /// Returns true if the end of an escape sequence is detected.
  bool _detectEscapeEnd(int code) {
    if (code == 109) {
      _inputSwitch = InputMode.normal;
      return true;
    }
    return false;
  }
  
  /// Appends a new [SpanElement] with the contents of [_outString]
  /// to the [_buffer] and updates the display.
  void _handleOutString(List<int> outString) {
    var codes = UTF8.decode(outString).codeUnits;
    for (var code in codes) {
      String char = new String.fromCharCode(code);
      if (code == 10) {
        _model.cursorNewLine();
        continue;
      }

      if (code == 13) {
        // TODO: figure out what to do with the carriage return since it
        // comes with the newline. Eat it for now.
        continue;
      }

      _model.setGlyphAt(new Glyph(char), _model.cursor.row, _model.cursor.col);
      _model.cursorNext();
    }

    refreshDisplay();
  }
  
  /// Placeholder for handling an escape sequence.
  void _setAttributeMode(List<int> escapeSequence) {
    print('setting attribute mode! ' + escapeSequence.toString());
  }
  
  /// Display initialization.
  void _initDisplay() {
    for (var i = 0; i < _rows; i++) {
      DivElement row = new DivElement();
      row.classes.add('termrow');

      for (var j = 0; j < _cols; j++) {
        row.innerHtml += "&nbsp";
      }
      
      div.children.add(row);
    }

    refreshDisplay();
  }
  
  void refreshDisplay() {
    for (int r = 0; r < _rows; r++) {
      String s = '';
      for (int c = 0; c < _cols; c++) {
        Glyph g = _model.getGlyphAt(r, c);
        s += g.value;
      }
      div.children[r].innerHtml = s;
    }
  }
}