library explorer_helpers;

import 'dart:html';

/// Helper function removes spaces from path and file name

  String removeSpaces(String raw){
    raw = raw.replaceAll(' ', '');
    return raw;
  }

  /// Function checks for nested folders

  bool checkContents(HtmlElement ele){
    var subFolder = false;
    var children;
    if(ele.hasChildNodes()){
      var ul = ele.childNodes;
      children = ul[ul.length-1].childNodes;
    }
    if(children != null){
      for(var item in children){
            if(item.dataset['isDir'] == 'true'){
              subFolder = true;
            }
          }
    }
    return subFolder;
  }

  /// Helper function to check for illegal drops to nested directories

  bool checkNested(String firstPath, String secondPath){
    var first = firstPath.split('/');
    var second = secondPath.split('/');
    int shortestPath;
    bool valid = false;
    first.length < second.length ? shortestPath = first.length : shortestPath = second.length;
    for(int i = 0; i < shortestPath; i++) {
      if(first[i] != second[i]) {
        valid = true;
      }
    }
    return valid;
  }

