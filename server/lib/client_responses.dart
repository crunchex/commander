library client_responses;

import 'dart:io';
import 'server_helper.dart' as help;
import 'console_parser.dart';

void sendDirectory(WebSocket s, Directory dir) {
  help.getDirectory(dir).then((files) {
    s.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
  });
}

void sendPath(WebSocket s, Directory dir) {
  help.formattedMessage(s, 'EXPLORER_DIRECTORY_PATH', dir.path);
}

void sendFileContents(WebSocket s, String path) {
  var fileToOpen = new File(path);
  fileToOpen.readAsString().then((String contents) {
    s.add('[[EDITOR_FILE_TEXT]]' + contents);
  });
}

void saveFile(String args) {
  // List[0] = data, List[1] = path.
  List<String> argsList = args.split('[[PATH]]');

  var fileToSave = new File(argsList[1]);
  fileToSave.writeAsString(argsList[0]);
}

void fsNewFile(String path) {
  var newFile = new File(path);
  newFile.create();
}

void fsNewFolder(String path) {
  var newFolder = new Directory(path);
  newFolder.createSync();
}

void fsRename(String rename) {
  List<String> renameList = rename.split(' ');
  
  if (!FileSystemEntity.isDirectorySync(renameList[0])) {
    var fileToRename = new File(renameList[0]);
    fileToRename.rename(renameList[1]);
  } else {
    var dirToRename = new Directory(renameList[0]);
    dirToRename.rename(renameList[1]);
  }
}

void fsDelete(String path) {
  // Can't simply just create a FileSystemEntity and delete it, since
  // it is an abstract class. This is a dumb way to create the proper
  // entity class.
  try {
    var dirToDelete = new Directory(path);
    dirToDelete.delete(recursive:true);
  } catch (e) {
    var fileToDelete = new File(path);
    fileToDelete.delete();
  }
}

void processCommand(WebSocket s, String command) {
  List args = parseCommandInput(command);
  Process.run(args[0], args[1]).then((ProcessResult results) {
    s.add('[[CONSOLE_COMMAND]]' + results.stdout);
  });
}