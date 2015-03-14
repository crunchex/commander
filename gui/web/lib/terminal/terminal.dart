library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:quiver/core.dart';

part 'model.dart';
part 'input_keys.dart';
part 'theme.dart';

/// A class for rendering a terminal emulator in a [DivElement] (param).
/// [stdout] needs to receive individual UTF8 integers and will handle
/// them appropriately.
class Terminal {
  /// The [DivElement] within which all [Terminal] graphical elements
  /// are rendered.
  DivElement div;

  /// A stream of [String], JSON-encoded UTF8 bytes (List<int>).
  StreamController<List<int>> stdout;

  /// A stream of [String], JSON-encoded UTF8 bytes (List<int>).
  StreamController<List<int>> stdin;

  /// An int that sets the number of lines scrolled per mouse
  /// wheel event. Default: 3
  int scrollSpeed = 3;

  // Private
  int _charWidth, _charHeight;
  List<SpanElement> _buffer;
  Model _model;
  DisplayAttributes _attr;
  bool _inputDone;
  List<int> _inputString;
  Theme _theme;

  static const int ESC = 27;

  Terminal (this.div) {
    stdout = new StreamController<List<int>>();
    stdin = new StreamController<List<int>>();
    _inputDone = false;
    _inputString = [];

    _charWidth = 7;
    _charHeight = 14;
    _model = new Model(_rows, _cols);
    _attr = new DisplayAttributes();
    _theme = new Theme.SolarizedDark();

    _registerEventHandlers();
  }

  // TODO: fix this dynamic size detection
  //int get _cols => (div.borderEdge.width - 10) ~/ _charWidth - 1;
  //int get _rows => (div.borderEdge.height - 10) ~/ _charHeight - 1;
  int get _cols => 80;
  int get _rows => 30;

  /// A [String] that sets the colored theme of the entire [Terminal].
  /// Supported themes: solarized-dark, solarized-light.
  /// Default: solarized-dark.
  void set theme(String name) {
    switch (name) {
      case 'solarized-light':
        _theme = new Theme.SolarizedLight();
        break;
      default:
        _theme = new Theme.SolarizedDark();
    }
    div.style.backgroundColor = _theme.backgroundColor;
    _refreshDisplay();
  }

  void _registerEventHandlers() {
    stdout.stream.listen((List<int> out) => _processStdOut(new List.from(out)));

    div.onKeyUp.listen((e) => _handleInput(e));

    // Disable browser navigation keys.
    div.onKeyDown.listen((e) {
      if (e.keyCode == 8 || e.keyCode == 9) e.preventDefault();
    });

    div.onMouseWheel.listen((wheelEvent) {
      // Scrolling should target only the console.
      wheelEvent.preventDefault();

      (wheelEvent.deltaY < 0) ? _model.scrollUp(scrollSpeed) : _model.scrollDown(scrollSpeed);
      _refreshDisplay();
    });

    div.onPaste.listen((e) {
      String pasteString = e.clipboardData.getData('text');
      for (int i in pasteString.runes) {
        stdin.add([i]);
      }
    });
  }

  /// Handles a given [KeyboardEvent].
  void _handleInput(KeyboardEvent e) {
    int key = e.keyCode;

    // Eat ctrl-c and ctrl-v (copy & paste).
    if (e.ctrlKey) {
      if (key == 86 || key == 67) return;
    }

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
    }

    // Don't let solo modifier keys through (Shift=16, Ctrl=17, Meta=91, Alt=18).
    if (key != 16 && key != 17 && key != 91 && key != 18) {
      stdin.add([key]);
    }
  }

  /// Splits a UTF8 string into substrings, split by preceding escape sequences.
  void _processStdOut(List<int> output) {
    List<int> escapeString, escape, string;
    int start, end;

    // Handle cases where there are no Escapes, or a regular string
    // precedes an Escape.
    end = output.indexOf(ESC);
    if (end == -1) {
      _handleOutString(output);
      return;
    } else {
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
  }

  /// Appends a new [SpanElement] with the contents of [_outString]
  /// to the [_buffer] and updates the display.
  void _handleOutString(List<int> string) {
    var codes = UTF8.decode(string).codeUnits;
    for (var code in codes) {
      String char = new String.fromCharCode(code);
      if (code == 10) {
        _drawSpace();
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

      Glyph g = new Glyph(char, _attr);
      _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
      _model.cursorNext();
    }

    _drawCursor();
    _refreshDisplay();
  }

  /// Sets local [DisplayAttributes], given [escape].
  void _setAttributeMode(List<int> escape) {
    String decodedEsc = UTF8.decode(escape);

    if (decodedEsc.contains('0m')) {
      _attr.resetAll();
    }

    if (decodedEsc.contains(';1')) _attr.bright = true;
    if (decodedEsc.contains(';2')) _attr.dim = true;
    if (decodedEsc.contains(';4')) _attr.underscore = true;
    if (decodedEsc.contains(';5')) _attr.blink = true;
    if (decodedEsc.contains(';7')) _attr.reverse = true;
    if (decodedEsc.contains(';8')) _attr.hidden = true;

    if (decodedEsc.contains(';30')) _attr.fgColor = 'black';
    if (decodedEsc.contains(';31')) _attr.fgColor = 'red';
    if (decodedEsc.contains(';32')) _attr.fgColor = 'green';
    if (decodedEsc.contains(';33')) _attr.fgColor = 'yellow';
    if (decodedEsc.contains(';34')) _attr.fgColor = 'blue';
    if (decodedEsc.contains(';35')) _attr.fgColor = 'magenta';
    if (decodedEsc.contains(';36')) _attr.fgColor = 'cyan';
    if (decodedEsc.contains(';37')) _attr.fgColor = 'white';

    if (decodedEsc.contains(';40')) _attr.bgColor = 'black';
    if (decodedEsc.contains(';41')) _attr.bgColor = 'red';
    if (decodedEsc.contains(';42')) _attr.bgColor = 'green';
    if (decodedEsc.contains(';43')) _attr.bgColor = 'yellow';
    if (decodedEsc.contains(';44')) _attr.bgColor = 'blue';
    if (decodedEsc.contains(';45')) _attr.bgColor = 'magenta';
    if (decodedEsc.contains(';46')) _attr.bgColor = 'cyan';
    if (decodedEsc.contains(';47')) _attr.bgColor = 'white';
  }

  /// Renders the cursor at [Cursor]'s current position.
  void _drawCursor() {
    Glyph cursor = new Glyph('|', _attr);
    _model.setGlyphAt(cursor, _model.cursor.row, _model.cursor.col);
  }

  /// Renders a space at [Cursor]'s current position.
  /// Useful for "removing" the cursor.
  void _drawSpace() {
    Glyph cursor = new Glyph(Glyph.SPACE, _attr);
    _model.setGlyphAt(cursor, _model.cursor.row, _model.cursor.col);
  }

  /// Generates the HTML for an individual row given
  /// the [Glyph]s contained in the model at that
  /// corresponding row.
  DivElement _generateRow(int r) {
    Glyph prev, curr;

    DivElement row = new DivElement();

    SpanElement span = new SpanElement();
    prev = _model.getGlyphAt(r, 0);

    span.style.color = _theme.colors[prev.fgColor];
    span.style.backgroundColor = _theme.colors[prev.bgColor];
    span.text += prev.value;

    for (int c = 1; c < _cols; c++) {
      curr = _model.getGlyphAt(r, c);

      if (curr != prev || c == _cols - 1) {
        row.append(span);

        // TODO: handle other display attributes, like blink.
        span = new SpanElement();
        span.style.color = _theme.colors[curr.fgColor];
        span.style.backgroundColor = _theme.colors[curr.bgColor];
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
  void _refreshDisplay() {
    div.innerHtml = '';

    DivElement row;
    for (int r = 0; r < _rows; r++) {
      row = _generateRow(r);
      row.classes.add('termrow');

      div.append(row);
    }
  }
}