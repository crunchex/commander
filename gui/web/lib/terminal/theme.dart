part of terminal;

/// A class for encapsulating various color themes
/// for [Terminal];
class Theme {
  Map colors;
  String backgroundColor;
  
  Theme.SolarizedDark() {
    colors = {
      'black'   : '#002b36',
      'red'     : '#dc322f',
      'green'   : '#859900',
      'yellow'  : '#b58900',
      'blue'    : '#268bd2',
      'magenta' : '#d33682',
      'cyan'    : '#2aa198',
      'white'   : '#93a1a1'
    };
    
    backgroundColor = '#002b36';
  }
  
  Theme.SolarizedLight() {
    colors = {
      'black'   : '#fdf6e3',
      'red'     : '#dc322f',
      'green'   : '#859900',
      'yellow'  : '#b58900',
      'blue'    : '#268bd2',
      'magenta' : '#d33682',
      'cyan'    : '#2aa198',
      'white'   : '#586e75'
    };
    
    backgroundColor = '#fdf6e3';
  }
}