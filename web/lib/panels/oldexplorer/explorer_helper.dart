part of updroid_explorer;

/// Helper function removes spaces from path and file name
String removeSpaces(String raw) {
  return raw.replaceAll(' ', '');
}

/// Function checks for nested folders
bool checkContents(HtmlElement e) {
  var subFolder = false;
  var children;
  if (e.hasChildNodes()) {
    var ul = e.childNodes;
    children = ul[ul.length - 1].childNodes;
  }
  if (children != null) {
    for (var item in children) {
      if (item.dataset['isDir'] == 'true') {
        subFolder = true;
      }
    }
  }
  return subFolder;
}

/// Helper function to check for illegal drops to nested directories
bool checkNested(String firstPath, String secondPath) {
  var first = firstPath.split('/');
  var second = secondPath.split('/');
  int shortestPath = first.length < second.length ? first.length : second.length;

  bool valid = false;
  for (int i = 0; i < shortestPath; i++) {
    if (first[i] != second[i]) {
      valid = true;
    }
  }
  return valid;
}
