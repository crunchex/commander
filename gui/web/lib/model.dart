part of terminal;

/// The data model class for an individual glyph within Model.
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
  
  Glyph (this.value, [this.bright = false, this.dim = false, this.underscore = false,
         this.blink = false, this.reverse = false, this.hidden = false,
         this.fgColor = 'white', this.bgColor = 'white']);
  
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