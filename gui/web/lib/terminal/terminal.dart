library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:quiver/core.dart';

part 'model.dart';
part 'input_keys.dart';
part 'theme.dart';
part 'escape_sequences.dart';

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
  EscapeHandler _escHandler;

  static const int ESC = 27;

  Terminal (this.div) {
    stdout = new StreamController<List<int>>();
    stdin = new StreamController<List<int>>();
    _inputDone = false;
    _inputString = [];

    _charWidth = 7;
    _charHeight = 13;
    _model = new Model(_rows, _cols);
    _attr = new DisplayAttributes();
    _escHandler = new EscapeHandler(_model, _attr);
    _theme = new Theme.SolarizedDark();

    _registerEventHandlers();
  }

  // TODO: fix this dynamic size detection
  //int get _cols => (div.borderEdge.width - 10) ~/ _charWidth - 1;
  //int get _rows => (div.borderEdge.height - 10) ~/ _charHeight - 1;
  // _cols must be $COLUMNS + 1 or we see some glitchy stuff.
  int get _cols => 81;
  int get _rows => 33;

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
      if (e.keyCode == 8 || e.keyCode == 9 || e.keyCode == 32) e.preventDefault();
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

  /// Processes [output] by coordinating handling of strings
  /// and escape parsing.
  void _processStdOut(List<int> output) {
    int nextEsc;
    while (output.isNotEmpty) {
      nextEsc = output.indexOf(ESC);
      if (nextEsc == -1) {
        _handleOutString(output);
        return;
      } else {
        _handleOutString(output.sublist(0, nextEsc));
        output = _parseEscape(output.sublist(nextEsc));
      }
    }
  }

  /// Parses out escape sequences. When it finds one,
  /// it handles it and returns the remainder of [output].
  List<int> _parseEscape(List<int> output) {
    List<int> escape;
    int termIndex;
    for (int i = 1; i <= output.length; i++) {
      termIndex = i;
      escape = output.sublist(0, i);

      if (escape.length != 1 && escape.last == 27) {
        print('Unknown escape detected: ${escape.sublist(0, escape.length - 1).toString()}');
        break;
      }

      String encodedEscape = JSON.encode(escape);
      if (EscapeHandler.constantEscapes.containsKey(encodedEscape)) {
        switch (EscapeHandler.constantEscapes[encodedEscape]) {
          case 'Erase End of Line':
            _escHandler.eraseEndOfLine();
            break;
          default:
            print('Constant escape : ${EscapeHandler
                .constantEscapes[encodedEscape]} (${escape.toString()}) not yet supported');
        }
        _refreshDisplay();
        break;
      }

      if (EscapeHandler.variableEscapeTerminators.containsKey(escape.last)) {
        switch (EscapeHandler
                .variableEscapeTerminators[escape.last]) {
          case 'Set Attribute Mode':
            _escHandler.setAttributeMode(escape);
            break;
          case 'Cursor Home':
            _escHandler.cursorHome(escape);
            break;
          case 'Cursor Forward':
            _escHandler.cursorForward();
            break;
          default:
            print('Variable escape : ${EscapeHandler
                .variableEscapeTerminators[escape.last]} (${escape.toString()}) not yet supported');
        }
        _refreshDisplay();
        break;
      }
    }

    return output.sublist(termIndex);
  }

  /// Appends a new [SpanElement] with the contents of [_outString]
  /// to the [_buffer] and updates the display.
  void _handleOutString(List<int> string) {
    var codes = UTF8.decode(string).codeUnits;
    var prevCode;
    for (var code in codes) {
      String char = new String.fromCharCode(code);

      if (code == 13) {
        _model.cursorCarriageReturn();
        prevCode = code;
        continue;
      }

      if (code == 10) {
        _model.cursorNewLine();
        prevCode = code;
        continue;
      }

      if (code == 32) {
        char = Glyph.SPACE;
      }

      if (code == 8) {
        prevCode = code;
        continue;
      }

      Glyph g = new Glyph(char, _attr);
      _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
      _model.cursorForward();

      prevCode = code;
    }

    _refreshDisplay();
  }

  /// Generates the HTML for an individual row given
  /// the [Glyph]s contained in the model at that
  /// corresponding row.
  DivElement _generateRow(int r) {
    Glyph prev, curr;

    DivElement row = new DivElement();
    String str = '';
    prev = _model.getGlyphAt(r, 0);
    for (int c = 0; c < _cols; c++) {
      curr = _model.getGlyphAt(r, c);

      if (!curr.hasSameAttributes(prev) || c == _cols - 1) {
        if (prev.hasDefaults()) {
          row.append(new DocumentFragment.html(str));
        } else {
          SpanElement span = new SpanElement();
          span.style.color = _theme.colors[prev.fgColor];
          span.style.backgroundColor = _theme.colors[prev.bgColor];
          span.append(new DocumentFragment.html(str));
          row.append(span);
        }

        str = '';
      }

      // Draw the cursor.
      if (_model.cursor.row == r && _model.cursor.col == c) {
        str += '|';
      } else {
        str += curr.value;
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
