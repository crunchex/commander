library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:quiver/core.dart';
import 'console_helper.dart';

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
  StreamController stdin;
  StreamController stdout;
  WebSocket ws;

  // Private
  int _charWidth, _charHeight, _inputSwitch;
  List<int> _cursorXY;
  List<SpanElement> _buffer;
  Model _model;
  DisplayAttributes _attributes;
  bool inputDone;
  List<int> inputString;
  
  static const int ESC = 27;
  
  Terminal (this.div, this.ws) {
    stdin = new StreamController<String>();
    stdout = new StreamController<String>();
    inputDone = false;
    inputString = [];

    _charWidth = 7;
    _charHeight = 14;
    _inputSwitch = InputMode.normal;
    _model = new Model(_rows, _cols);
    _attributes = new DisplayAttributes();
    
    _registerEventHandlers();
  }
  
  int get _cols => (div.borderEdge.width - 10) ~/ _charWidth - 1;
  int get _rows => (div.borderEdge.height - 10) ~/ _charHeight - 1;
  
  void _registerEventHandlers() {
    stdout.stream.listen((String out) => processStdOut(JSON.decode(out)));
    stdin.stream.listen((int input) => processStdIn(input));
    
    div.onKeyUp.listen((e) => handleInput(e));
    
    div.onKeyDown.listen((e) {
      if (e.keyCode == 8) e.preventDefault();
    });
    
    div.onMouseWheel.listen((wheelEvent) {
      // Scrolling should target only the console.
      wheelEvent.preventDefault();
      
      if (wheelEvent.deltaY < 0) {
        scrollUp();
      } else {
        scrollDown();
      }
    });
  }
  
  void handleInput(KeyboardEvent e) {
    int key = e.keyCode;
    
    // keyCode behaves very oddly.
    if (!e.shiftKey) {
      if (NOSHIFT_KEYS.containsKey(key)) {
        key = NOSHIFT_KEYS[key];
      }
    } else {
      if (SHIFT_KEYS.containsKey(key)) {
        key = SHIFT_KEYS[key];
      }
    }

    // Carriage Return (13) => New Line (10).
    if (key == 13) {
      key = 10;
      inputDone = true;
    }

    // Don't let solo modifier keys through (Shift=16, Ctrl=17, Meta=91, Alt=18).
    if (key != 16 && key != 17 && key != 91 && key != 18) {
      stdin.add(key);
      if (key == 8 && inputString.isNotEmpty) {
        inputString.removeLast();
      } else {
        inputString.add(key);
      }
    }
    
    if (inputDone) {
      ws.send('[[CONSOLE_INPUT]]' + JSON.encode(inputString));
      inputString = [];
      inputDone = false;
    }
  }
  
  /// Handles a scroll up action by relaying the command to the model
  /// and refreshing the display.
  void scrollUp() {
    _model.scrollUp();
    refreshDisplay();
  }
  
  /// Handles a scroll down action by relaying the command to the model
  /// and refreshing the display.
  void scrollDown() {
    _model.scrollDown();
    refreshDisplay();
  }
  
  void processStdIn(int input) {
    String char = new String.fromCharCode(input);
    if (input == 10) {
      _model.cursorNewLine();
      _model.inputCursorIndex = 0;
      return;
    }

    if (input == 8 && _model.inputCursorIndex > 0) {
      _model.cursorBack();
      Glyph g = new Glyph(Glyph.SPACE, bright: _attributes.bright,
                                dim: _attributes.dim,
                                underscore: _attributes.underscore,
                                blink: _attributes.blink,
                                reverse: _attributes.reverse,
                                hidden: _attributes.hidden,
                                fgColor: _attributes.fgColor,
                                bgColor: _attributes.bgColor);
      _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
      refreshDisplay();
      return;
    }
    
    if (input == 32) {
      char = Glyph.SPACE;
    }

    Glyph g = new Glyph(char, bright: _attributes.bright,
                              dim: _attributes.dim,
                              underscore: _attributes.underscore,
                              blink: _attributes.blink,
                              reverse: _attributes.reverse,
                              hidden: _attributes.hidden,
                              fgColor: _attributes.fgColor,
                              bgColor: _attributes.bgColor);
    _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
    _model.cursorNext();
    _model.inputCursorIndex++;

    refreshDisplay();
  }
  
  /// Splits a UTF8 string into substrings, split by preceding escape sequences.
  void processStdOut(List<int> output) {
    List<int> escapeString, escape, string;
    int start, end;
    
    // The case where the current output contains
    // no escape sequence.
    if (!output.contains(ESC)) {
      _handleOutString(output);
      return;
    }
    
    // Handle any string preceding the first escape sequence.
    if (output[0] != ESC) {
      end = output.indexOf(ESC);
      string = output.sublist(0, end);
      _handleOutString(string);

      for (int i in string) {
        output.remove(i);
      }
    }

    // TODO: make escape parsing independent of the display attribute
    // terminator, 109 (m).
    while (true) {
      start = output.indexOf(ESC);
      List<int> subList = output.sublist(1);
      if (!subList.contains(ESC)) break;
      end = subList.indexOf(ESC) + 1;

      escapeString = output.sublist(start, end);
      escape = escapeString.sublist(0, escapeString.indexOf(109) + 1);
      string = escapeString.sublist(escapeString.indexOf(109) + 1);
      _setAttributeMode(escape);
      _handleOutString(string);

      for (int j in escapeString) {
        output.remove(j);
      }
    }

    // Deal with the remaining string composed of at least one final
    // escape.
    escape = output.sublist(0, output.indexOf(109) + 1);
    string = output.sublist(output.indexOf(109) + 1);
    _setAttributeMode(escape);
    _handleOutString(string);

    refreshDisplay();
  }
  
  /// Appends a new [SpanElement] with the contents of [_outString]
  /// to the [_buffer] and updates the display.
  void _handleOutString(List<int> string) {
    var codes = UTF8.decode(string).codeUnits;
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
      
      if (code == 32) {
        char = Glyph.SPACE;
      }

      Glyph g = new Glyph(char, bright: _attributes.bright,
                                dim: _attributes.dim,
                                underscore: _attributes.underscore,
                                blink: _attributes.blink,
                                reverse: _attributes.reverse,
                                hidden: _attributes.hidden,
                                fgColor: _attributes.fgColor,
                                bgColor: _attributes.bgColor);
      _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
      _model.cursorNext();
    }
  }
  
  /// Sets local [DisplayAttributes], given [escape].
  void _setAttributeMode(List<int> escape) {
    String decodedEsc = UTF8.decode(escape);

    if (escape.contains('0;')) {
      _attributes.resetAll(); 
    }

    if (decodedEsc.contains(';1')) _attributes.bright = true;
    if (decodedEsc.contains(';2')) _attributes.dim = true;
    if (decodedEsc.contains(';4')) _attributes.underscore = true;
    if (decodedEsc.contains(';5')) _attributes.blink = true;
    if (decodedEsc.contains(';7')) _attributes.reverse = true;
    if (decodedEsc.contains(';8')) _attributes.hidden = true;

    if (decodedEsc.contains(';30')) _attributes.fgColor = DisplayAttributes.COLORS[30];
    if (decodedEsc.contains(';31')) _attributes.fgColor = DisplayAttributes.COLORS[31];
    if (decodedEsc.contains(';32')) _attributes.fgColor = DisplayAttributes.COLORS[32];
    if (decodedEsc.contains(';33')) _attributes.fgColor = DisplayAttributes.COLORS[33];
    if (decodedEsc.contains(';34')) _attributes.fgColor = DisplayAttributes.COLORS[34];
    if (decodedEsc.contains(';35')) _attributes.fgColor = DisplayAttributes.COLORS[35];
    if (decodedEsc.contains(';36')) _attributes.fgColor = DisplayAttributes.COLORS[36];
    if (decodedEsc.contains(';37')) _attributes.fgColor = DisplayAttributes.COLORS[37];

    if (decodedEsc.contains(';40')) _attributes.bgColor = DisplayAttributes.COLORS[30];
    if (decodedEsc.contains(';41')) _attributes.bgColor = DisplayAttributes.COLORS[31];
    if (decodedEsc.contains(';42')) _attributes.bgColor = DisplayAttributes.COLORS[32];
    if (decodedEsc.contains(';43')) _attributes.bgColor = DisplayAttributes.COLORS[33];
    if (decodedEsc.contains(';44')) _attributes.bgColor = DisplayAttributes.COLORS[34];
    if (decodedEsc.contains(';45')) _attributes.bgColor = DisplayAttributes.COLORS[35];
    if (decodedEsc.contains(';46')) _attributes.bgColor = DisplayAttributes.COLORS[36];
    if (decodedEsc.contains(';47')) _attributes.bgColor = DisplayAttributes.COLORS[37];
  }

  /// Generates the HTML for an individual row given
  /// the [Glyph]s contained in the model at that
  /// corresponding row.
  DivElement generateRow(int r) {
    Glyph prev, curr;

    DivElement row = new DivElement();

    SpanElement span = new SpanElement();
    prev = _model.getGlyphAt(r, 0);

    span.style.color = prev.fgColor;
    span.style.backgroundColor = prev.bgColor;
    span.text += prev.value;

    for (int c = 1; c < _cols; c++) {
      curr = _model.getGlyphAt(r, c);
      
      if (curr != prev || c == _cols - 1) {
        row.append(span);

        // TODO: handle other display attributes, link blink.
        span = new SpanElement();
        span.style.color = curr.fgColor;
        span.style.backgroundColor = curr.bgColor;
        span.text += curr.value;
      } else {
        span.text += curr.value;
      }

      prev = curr;
    }

    return row;
  }
  
  /// Refreshes the entire console [DivElement] by setting its
  /// contents to null and regenerating each row [DivElement].
  void refreshDisplay() {
    div.innerHtml = '';
    
    DivElement row;
    for (int r = 0; r < _rows; r++) {
      row = generateRow(r);
      row.classes.add('termrow');
      
      div.append(row);
    }
  }
}