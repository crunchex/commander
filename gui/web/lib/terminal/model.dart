part of terminal;

class Cursor {
  int row = 0;
  int col = 0;

  String toString () {
    return 'row: $row, col: $col';
  }
}

/// Represents the data model for [Terminal].
class Model {
  Cursor cursor;
  int numRows, numCols;

  int inputCursorIndex;

  // Implemented as stacks in scrolling.
  List<List> _reverseBuffer;
  List<List> _forwardBuffer;

  // Implemented as a queue in scrolling.
  List<List> _rows;

  Model (this.numRows, this.numCols) {
    cursor = new Cursor();
    inputCursorIndex = 0;
    _reverseBuffer = [];
    _forwardBuffer = [];
    _rows = [];

    _initModel();
  }

  /// Returns the [Glyph] at row, col.
  Glyph getGlyphAt(int row, int col) => _rows[row][col];

  /// Sets a [Glyph] at location row, col.
  void setGlyphAt(Glyph g, int row, int col) {
    _rows[row][col] = g;
  }

  void cursorNext() {
    if (cursor.col < numCols - 1) {
      cursor.col++;
      return;
    }

    cursorNewLine();
  }

  void cursorBack() {
    if (inputCursorIndex > 0) {
      cursor.col--;
      inputCursorIndex--;
    }
  }

  void cursorCarriageReturn() {
    cursor.col = 0;
  }

  void cursorNewLine() {
    if (cursor.row < numRows - 1) {
      cursor.row++;
    } else {
      pushBuffer();
    }
  }

  void pushBuffer() {
    _reverseBuffer.add(_rows[0]);
    _rows.removeAt(0);

    List<Glyph> newRow = [];
    for (int c = 0; c < numCols; c++) {
      newRow.add(new Glyph(Glyph.SPACE, new DisplayAttributes()));
    }
    _rows.add(newRow);
  }

  /// Manipulates the buffers and rows to handle scrolling
  /// upward of a single line.
  void scrollUp(int numLines) {
    for (int i = 0; i < numLines; i++) {
      if (_reverseBuffer.isEmpty) return;

      _rows.insert(0, _reverseBuffer.last);
      _reverseBuffer.removeLast();
      _forwardBuffer.add(_rows[_rows.length - 1]);
      _rows.removeLast();
    }
  }

  /// Manipulates the buffers and rows to handle scrolling
  /// upward of a single line.
  void scrollDown(int numLines) {
    for (int i = 0; i < numLines; i++) {
      if (_forwardBuffer.isEmpty) return;

      _rows.add(_forwardBuffer.last);
      _forwardBuffer.removeLast();
      _reverseBuffer.add(_rows[0]);
      _rows.removeAt(0);
    }
  }

  /// Initializes the internal model with a List of Lists.
  /// Each location defaults to a Glyph.SPACE.
  void _initModel() {
    for (int r = 0; r < numRows; r++) {
      _rows.add(new List<Glyph>());
      for (int c = 0; c < numCols; c++) {
        _rows[r].add(new Glyph(Glyph.SPACE, new DisplayAttributes()));
      }
    }
  }

  /// Display the specified [member] for each [Glyph] in the model.
  void debugDisplay(String member) {
    for (int r = 0; r < numRows; r++) {
      String s = '';
      for (int c = 0; c < numCols; c++) {
        Glyph g = _rows[r][c];
        switch (member) {
          case 'value':
            s += '${g.value} ';
            break;
          case 'bright':
            s += '${g.bright} ';
            break;
          case 'dim':
            s += '${g.dim} ';
            break;
          case 'underscore':
            s += '${g.underscore} ';
            break;
          case 'blink':
            s += '${g.blink} ';
            break;
          case 'reverse':
            s += '${g.reverse} ';
            break;
          case 'hidden':
            s += '${g.hidden} ';
            break;
          case 'fgColor':
            s += '${g.fgColor} ';
            break;
          case 'bgColor':
            s += '${g.bgColor} ';
            break;
        }
      }
      print(s);
    }
  }
}

/// Holds the current state of [Terminal] display attributes.
class DisplayAttributes {
  bool bright, dim, underscore, blink, reverse, hidden;
  String fgColor, bgColor;

  DisplayAttributes ({this.bright: false, this.dim: false, this.underscore: false,
         this.blink: false, this.reverse: false, this.hidden: false,
         this.fgColor: 'white', this.bgColor: 'black'});

  String toString() {
    Map properties = {
      'bright': bright,
      'dim': dim,
      'underscore': underscore,
      'blink': blink,
      'reverse': reverse,
      'hidden': hidden,
      'fgColor': fgColor,
      'bgColor': bgColor
    };
    return JSON.encode(properties);
  }

  void resetAll() {
    bright = false;
    dim = false;
    underscore = false;
    blink = false;
    reverse = false;
    hidden = false;

    fgColor = 'white';
    bgColor = 'black';
  }
}

/// The data model class for an individual glyph within [Model].
class Glyph {
  static const SPACE = ' ';
  static const AMP = '&';
  static const LT = '<';
  static const GT = '>';

  bool bright, dim, underscore, blink, reverse, hidden;
  String value, fgColor, bgColor;

  Glyph (this.value, DisplayAttributes attr) {
    bright = attr.bright;
    dim = attr.dim;
    underscore = attr.underscore;
    blink = attr.blink;
    reverse = attr.reverse;
    hidden = attr.hidden;
    fgColor = attr.fgColor;
    bgColor = attr.bgColor;
  }

  operator ==(Glyph other) {
    return (value == other.value
            && bright == other.bright
            && dim == other.dim
            && underscore == other.underscore
            && blink == other.blink
            && reverse == other.reverse
            && hidden == other.hidden
            && fgColor == other.fgColor
            && bgColor == other.bgColor);
  }

  bool hasSameAttributes(Glyph other) {
    return (bright == other.bright
            && dim == other.dim
            && underscore == other.underscore
            && blink == other.blink
            && reverse == other.reverse
            && hidden == other.hidden
            && fgColor == other.fgColor
            && bgColor == other.bgColor);
  }

  bool hasDefaults() {
    return (bright == false
            && dim == false
            && underscore == false
            && blink == false
            && reverse == false
            && hidden == false
            && fgColor == 'white'
            && bgColor == 'black');
  }

  int get hashCode {
    List members = [bright, dim, underscore, blink, reverse, hidden, fgColor, bgColor];
    return hashObjects(members);
  }

  String toString() {
    Map properties = {
      'value': value,
      'bright': bright,
      'dim': dim,
      'underscore': underscore,
      'blink': blink,
      'reverse': reverse,
      'hidden': hidden,
      'fgColor': fgColor,
      'bgColor': bgColor
    };
    return JSON.encode(properties);
  }
}