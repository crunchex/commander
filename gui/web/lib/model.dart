part of terminal;

/// Represents the data model for [Terminal].
class Model {
  List<List> _model;
  List cursor;
  
  Model (int numRows, int numCols) {
    _model = new List(numRows);
    cursor = [0, 0];

    initModel(numCols);
  }
  
  /// Returns the [Glyph] at x,y.
  Glyph getGlyphAt(int x, int y) => _model[x][y];
  
  /// Sets a [Glyph] at location x,y.
  void setGlyphAt(Glyph g, int x, int y) {
    _model[x][y] = g;
  }
  
  void cursorNext() {
    if (cursor[1] == _model.length - 1) {
      cursor[1] = 0;
      if (cursor[0] == _model[0].length - 1) {
      cursor[0] = 0;
      } else {
        cursor[0]++;
      }
    } else {
      cursor[1]++;
    }
  }
  
  void cursorNewLine() {
    cursor[1] = 0;
    cursor[0] = cursor[0] == _model.length - 1 ? 0 : cursor[0] + 1;
  }
  
  /// Initializes the internal model with a List of Lists.
  /// Each location defaults to a Glyph.SPACE.
  void initModel(int numCols) {
    for (int x = 0; x < _model.length; x++) {
      _model[x] = new List<Glyph>(numCols);
      for (int y = 0; y < numCols; y++) {
        _model[x][y] = new Glyph(Glyph.SPACE);
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

class Cursor extends Glyph {
  Cursor () : super (Glyph.SPACE, blink:true);
}