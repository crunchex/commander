part of terminal;

Map constantEscapes = {
  // Device Status
  [27, 91, 99]: 'Query Device Code'
};

Map variableEscapeTerminators = {
  99: 'Report Device Code',
  82: 'Report Cursor Position',
  72: 'Cursor Home',
  65: 'Cursor Up',
  66: 'Cursor Down',
  67: 'Cursor Forward',
  68: 'Cursor Backward',
  102: 'Force Cursor Position',
  114: 'Scroll Screen',
  112: 'Set Key Definition',
  109: 'Set Attribute Mode'
};