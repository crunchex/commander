part of terminal;

class EscapeHandler {
  // Taken from: http://www.termsys.demon.co.uk/vtansi.htm
  static Map constantEscapes = {
    // Device Status
    JSON.encode([27, 91, 99]): 'Query Device Code',
    JSON.encode([27, 91, 53, 110]): 'Query Device Status',
    JSON.encode([27, 91, 48, 110]): 'Report Device OK',
    JSON.encode([27, 91, 51, 110]): 'Report Device Failure',
    JSON.encode([27, 91, 54, 110]): 'Query Cursor Position',
    JSON.encode([27, 91, 53, 110]): 'Report Cursor Position',
    // Terminal Setup
    JSON.encode([27, 99]): 'Reset Device',
    JSON.encode([27, 55, 104]): 'Enable Line Wrap',
    JSON.encode([27, 55, 108]): 'Disable Line Wrap',
    // Fonts
    JSON.encode([27, 40]): 'Font Set G0',
    JSON.encode([27, 41]): 'Font Set G1',
    // Cursor Control
    JSON.encode([27, 91, 115]): 'Save Cursor',
    JSON.encode([27, 91, 117]): 'Unsave Cursor',
    JSON.encode([27, 55]): 'Save Cursor & Attrs',
    JSON.encode([27, 56]): 'Restore Cursor & Attrs',
    // Scrolling
    JSON.encode([27, 91, 114]): 'Scroll Screen',
    JSON.encode([27, 68]): 'Scroll Down',
    JSON.encode([27, 77]): 'Scroll Up',
    // Tab Control
    JSON.encode([27, 72]): 'Set Tab',
    JSON.encode([27, 91, 103]): 'Clear Tab',
    JSON.encode([27, 91, 51, 103]): 'Clear All Tabs',
    // Erasing Text
    JSON.encode([27, 91, 75]): 'Erase End of Line',
    JSON.encode([27, 91, 49, 75]): 'Erase Start of Line',
    JSON.encode([27, 91, 50, 75]): 'Erase Line',
    JSON.encode([27, 91, 74]): 'Erase Down',
    JSON.encode([27, 91, 49, 74]): 'Erase Up',
    JSON.encode([27, 91, 50, 74]): 'Erase Screen',
    // Printing
    JSON.encode([27, 91, 105]): 'Print Screen',
    JSON.encode([27, 91, 49, 105]): 'Print Line',
    JSON.encode([27, 91, 52, 105]): 'Stop Print Log',
    JSON.encode([27, 91, 53, 105]): 'Start Print Log'
  };

  static Map variableEscapeTerminators = {
    // Device Status
    99: 'Report Device Code',
    82: 'Report Cursor Position',
    // Cursor Control
    72: 'Cursor Home',
    65: 'Cursor Up',
    66: 'Cursor Down',
    67: 'Cursor Forward',
    68: 'Cursor Backward',
    102: 'Force Cursor Position',
    // Scrolling
    114: 'Scroll Screen',
    // Define Key
    112: 'Set Key Definition',
    // Set Display Attribute
    109: 'Set Attribute Mode'
  };

  Model _model;
  DisplayAttributes _attr;

  EscapeHandler(Model model, DisplayAttributes attr) {
    _model = model;
    _attr = attr;
  }

  /// Sets the cursor position where subsequent text will begin.
  /// If no row/column parameters are provided (ie. <ESC>[H),
  /// the cursor will move to the home position, at the upper left of the screen.
  void cursorHome(List<int> escape) {
    if (escape.length == 3) {
      print('cursor home: 0 0');
      return;
    }

    int indexOfSemi = escape.indexOf(59);
    _model.cursor.row = int.parse(UTF8.decode(escape.sublist(2, indexOfSemi)));
    _model.cursor.col = int.parse(UTF8.decode(escape.sublist(indexOfSemi + 1, escape.length - 1)));
  }

  /// Moves the cursor forward by COUNT columns; the default count is 1.
  void cursorForward() {
    _model.cursorForward();
  }

  /// Erases from the current cursor position to the end of the current line.
  void eraseEndOfLine() {
    _model.cursorBackward();
    // TODO: need to be able to use the same display attributes from the previous Glyph,
    // but right now it's too cumbersome.
    Glyph g = new Glyph(Glyph.SPACE, _attr);
    _model.setGlyphAt(g, _model.cursor.row, _model.cursor.col);
  }

  /// Sets multiple display attribute settings.
  /// Sets local [DisplayAttributes], given [escape].
  void setAttributeMode(List<int> escape) {
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
}