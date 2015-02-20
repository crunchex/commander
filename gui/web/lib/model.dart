part of terminal;

class Cursor {
  int row = 0;
  int col = 0;
}

/// Represents the data model for [Terminal].
class Model {
  List<List> _rows;
  Cursor cursor;
  
  Model (int numRows, int numCols) {
    _rows = new List(numRows);
    cursor = new Cursor();

    initModel(numCols);
  }
  
  /// Returns the [Glyph] at row, col.
  Glyph getGlyphAt(int row, int col) => _rows[row][col];
  
  /// Sets a [Glyph] at location row, col.
  void setGlyphAt(Glyph g, int row, int col) {
    _rows[row][col] = g;
  }
  
  void cursorNext() {
    if (cursor.col < _rows[0].length - 1) {
      cursor.col++;
      return;
    }
    
    cursorNewLine();
  }
  
  void cursorNewLine() {
    cursor.row = (cursor.row < _rows.length - 1) ? cursor.row + 1 : 0;
    cursor.col = 0;
  }
  
  /// Initializes the internal model with a List of Lists.
  /// Each location defaults to a Glyph.SPACE.
  void initModel(int numCols) {
    for (int r = 0; r < _rows.length; r++) {
      _rows[r] = new List<Glyph>(numCols);
      for (int c = 0; c < numCols; c++) {
        _rows[r][c] = new Glyph(Glyph.SPACE);
      }
    }
  }
}

/// The data model class for an individual glyph within [Model].
class Glyph {
  static const SPACE = '&nbsp;';
  static const AMP = '&amp;';
  static const LT = '&lt;';
  static const GT = '&gt;';
  static const COLORS = const {
    30: 'black',
    31: 'red',
    32: 'green',
    33: 'yellow',
    34: 'blue',
    35: 'magenta',
    36: 'cyan',
    37: 'white'
  };

  bool bright, dim, underscore, blink, reverse, hidden;
  String value,fgColor, bgColor;
  
  Glyph (this.value, {this.bright: false, this.dim: false, this.underscore: false,
         this.blink: false, this.reverse: false, this.hidden: false,
         this.fgColor: 'white', this.bgColor: 'white'});
  
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
  
  void resetAll() {
    bright = false;
    dim = false;
    underscore = false;
    blink = false;
    reverse = false;
    hidden = false;
    
    fgColor = COLORS[37];
    bgColor = COLORS[37];
  }
}