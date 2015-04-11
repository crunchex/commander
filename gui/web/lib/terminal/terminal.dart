library terminal;

import 'dart:html';
import 'dart:async';
import 'dart:convert';
import 'package:quiver/core.dart';

part 'model.dart';
part 'input_keys.dart';
part 'theme.dart';
part 'escape_handler.dart';

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

  /// Enable cursor blink. Default: true
  void set cursorBlink(bool b) {
    _cursorBlink = b;

    cancelBlink();
    setUpBlink();
    _refreshDisplay();
  }

  /// A [String] that sets the colored theme of the entire [Terminal].
  /// Supported themes: solarized-dark, solarized-light.
  /// Default: solarized-dark.
  void set theme(Theme thm) {
    _theme = thm;
    div.style.backgroundColor = _theme.backgroundColor;
    div.style.color = _theme.colors['white'];
    _refreshDisplay();
  }

  double charWidth;
  int charHeight;

  // Private
  Model _model;
  DisplayAttributes _currAttributes;
  Theme _theme;
  Timer _blinkTimer, _blinkTimeout;
  bool _blinkOn;
  bool _cursorBlink = true;

  static const int ESC = 27;

  Terminal (this.div) {
    stdout = new StreamController<List<int>>();
    stdin = new StreamController<List<int>>();

    _model = new Model(rows, cols);
    _currAttributes = new DisplayAttributes();
    _theme = new Theme.SolarizedDark();
    _blinkOn = false;
    charWidth = 6.778;
    charHeight = 13;

    setUpBlink();

    _registerEventHandlers();
  }

  void resize(int newCols, int newRows) {
    _model.resize(newCols, newRows);
    _refreshDisplay();
  }

  void setUpBlink() {
    if (!_cursorBlink) return;

    _blinkTimeout = new Timer(new Duration(milliseconds: 1000), () {
      _blinkTimer = new Timer.periodic(new Duration(milliseconds: 500), (timer) {
        _blinkOn = !_blinkOn;
        _refreshDisplay();
      });
    });
  }

  void cancelBlink() {
    if (_blinkTimeout != null) _blinkTimeout.cancel();
    if (_blinkTimer != null) _blinkTimer.cancel();
  }

  // TODO: fix this dynamic size detection. _charWidth = 7, _charWidth = 13.
//  int get cols {
//    if (div == null) return 80;
//    int numCols;
//    while (div.borderEdge.width < 0) {
//      numCols = (div.borderEdge.width - 10) ~/ _charWidth;
//    }
//    return numCols;
//  }
//
//  int get rows {
//    if (div == null) return 25;
//    int numRows;
//    while (div.borderEdge.height < 0) {
//      numRows = (div.borderEdge.height - 10) ~/ _charHeight;
//    }
//    return numRows;
//  }
  // _cols must be $COLUMNS + 1 or we see some glitchy stuff.
  int get cols => 58;
  int get rows => 31;

  void _registerEventHandlers() {
    stdout.stream.listen((List<int> out) => _processStdOut(new List.from(out)));

    div.onKeyUp.listen((e) => _handleInput(e));

    // Disable browser navigation keys.
    div.onKeyDown.listen((e) {
      //if (e.keyCode == 8 ||
      //    e.keyCode == 9 ||
      //    e.keyCode == 32 ||
      //    e.keyCode == 9) e.preventDefault();
      e.preventDefault();
    });

    div.onMouseWheel.listen((wheelEvent) {
      // Scrolling should target only the console.
      wheelEvent.preventDefault();

      cursorBlink = (_model.atBottom) ? true : false;
     _blinkOn = false;
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
    // Deactivate blinking while the user is typing.
    // Reactivate after an idle period.
    cancelBlink();
    _blinkOn = true;
    _model.scrollToBottom();
    setUpBlink();

    int key = e.keyCode;

    if (e.ctrlKey) {
      // Eat ctrl-v (paste).
      if (key == 86) return;

      if (key == 67) {
        key = 3;
      }

    }

    if (key == 38) {
      stdin.add([27, 91, 65]);
      return;
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

      bool escapeHandled = EscapeHandler.handleEscape(escape, _model, _currAttributes);
      if (escapeHandled) {
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
    for (var code in codes) {
      String char = new String.fromCharCode(code);

      if (code == 13) {
        _model.cursorCarriageReturn();
        continue;
      }

      if (code == 10) {
        _model.cursorNewLine();
        continue;
      }

      if (code == 32) {
        char = Glyph.SPACE;
      }

      if (code == 60) {
        char = Glyph.LT;
      }

      if (code == 62) {
        char = Glyph.GT;
      }

      if (code == 38) {
        char = Glyph.AMP;
      }

      if (code == 8 || code == 7) {
        continue;
      }

      Glyph g = new Glyph(char, _currAttributes);
      _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
      _model.cursorForward();
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
    for (int c = 0; c < cols; c++) {
      curr = _model.getGlyphAt(r, c);

      if (!curr.hasSameAttributes(prev) || c == cols - 1) {
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
      if (_model.cursor.row == r && _model.cursor.col == c && _blinkOn) {
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
    for (int r = 0; r < rows; r++) {
      row = _generateRow(r);
      row.classes.add('termrow');

      div.append(row);
    }
  }
}
