library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:quiver/core.dart';

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
  DisplayAttributes _attributes;
  
  static const int ESC = 27;
  
  Terminal (this.div) {
    stdout = new StreamController<String>();

    _charWidth = 11;
    _charHeight = 14;
    _inputSwitch = InputMode.normal;
    _model = new Model(_rows, _cols);
    _attributes = new DisplayAttributes();
    
    _registerEventHandlers();
  }
  
  int get _cols => div.borderEdge.width ~/ _charWidth - 1;
  int get _rows => div.borderEdge.height ~/ _charHeight - 1;
  
  void _registerEventHandlers() {
    stdout.stream.listen((String out) => processStdOut(JSON.decode(out)));
  }
  
  /// Splits a UTF8 string into substrings, split by preceding escape sequences.
  void processStdOut(List<int> output) {
    List<int> escapeString, escape, string;
    int start, end;

    while (true) {
      start = output.indexOf(ESC);

      List<int> subList = output.sublist(1);
      if (!subList.contains(ESC)) break;

      end = subList.indexOf(ESC) + 1;

      escapeString = output.sublist(start, end);
      escape = escapeString.sublist(0, escapeString.indexOf(109) + 1);
      string = escapeString.sublist(escapeString.indexOf(109) + 1);
      _handleOutString(escape, string);

      for (int j in escapeString) {
        output.remove(j);
      }
    }

    // Deal with the remaining string.
    escapeString = output.sublist(start, end);
    escape = escapeString.sublist(0, escapeString.lastIndexOf(109) + 1);
    string = escapeString.sublist(escapeString.lastIndexOf(109) + 1);
    _handleOutString(escape, string);

    refreshDisplay();
  }
  
  /// Appends a new [SpanElement] with the contents of [_outString]
  /// to the [_buffer] and updates the display.
  void _handleOutString(List<int> escape, List<int> string) {
    _setAttributeMode(escape);

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