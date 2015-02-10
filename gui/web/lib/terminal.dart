library terminal;

class Terminal {
  int cols, rows;
  List cursorXY;
  
  Terminal () {
    cols = 80;
    rows = 25;
  }
}