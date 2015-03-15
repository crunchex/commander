part of terminal;

// Taken from: http://www.termsys.demon.co.uk/vtansi.htm

Map constantEscapes = {
  // Device Status
  [27, 91, 99]: 'Query Device Code',
  [27, 91, 53, 110]: 'Query Device Status',
  [27, 91, 48, 110]: 'Report Device OK',
  [27, 91, 51, 110]: 'Report Device Failure',
  [27, 91, 54, 110]: 'Query Cursor Position',
  [27, 91, 53, 110]: 'Report Cursor Position',
  // Terminal Setup
  [27, 99]: 'Reset Device',
  [27, 55, 104]: 'Enable Line Wrap',
  [27, 55, 108]: 'Disable Line Wrap',
  // Fonts
  [27, 40]: 'Font Set G0',
  [27, 41]: 'Font Set G1',
  // Cursor Control
  [27, 91, 115]: 'Save Cursor',
  [27, 91, 117]: 'Unsave Cursor',
  [27, 55]: 'Save Cursor & Attrs',
  [27, 56]: 'Restore Cursor & Attrs',
  // Scrolling
  [27, 91, 114]: 'Scroll Screen',
  [27, 68]: 'Scroll Down',
  [27, 77]: 'Scroll Up',
  // Tab Control
  [27, 72]: 'Set Tab',
  [27, 91, 103]: 'Clear Tab',
  [27, 91, 51, 103]: 'Clear All Tabs',
  // Erasing Text
  [27, 91, 75]: 'Erase End of Line',
  [27, 91, 49, 75]: 'Erase Start of Line',
  [27, 91, 50, 75]: 'Erase Line',
  [27, 91, 74]: 'Erase Down',
  [27, 91, 49, 74]: 'Erase Up',
  [27, 91, 50, 74]: 'Erase Screen',
  // Printing
  [27, 91, 105]: 'Print Screen',
  [27, 91, 49, 105]: 'Print Line',
  [27, 91, 52, 105]: 'Stop Print Log',
  [27, 91, 53, 105]: 'Start Print Log'
};

Map variableEscapeTerminators = {
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