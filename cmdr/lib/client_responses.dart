library client_responses;

import 'dart:io';
import 'server_helper.dart' as help;

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
    s.add('[[EDITOR_FILE_TEXT]]' + path + '[[CONTENTS]]' + contents);
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

// fixpoint
// Need to make it able to grab files with spaces

// file1 file2

void fsRename(String rename) {
  
  List<String> renameList = rename.split(':divider:');
  
  if (!FileSystemEntity.isDirectorySync(renameList[0])) {
    var fileToRename = new File(renameList[0]);
    fileToRename.rename(renameList[1]);
  } else {
    var dirToRename = new Directory(renameList[0]);
    dirToRename.rename(renameList[1]);
  }
}

void fsDelete(String path, WebSocket socket) {
  // Can't simply just create a FileSystemEntity and delete it, since
  // it is an abstract class. This is a dumb way to create the proper
  // entity class.
  try {
    var dirToDelete = new Directory(path);
    dirToDelete.delete(recursive:true).then((path){
    //  sendDirectory(socket, dir);
    });
  } catch (e) {
    var fileToDelete = new File(path);
    fileToDelete.delete();
  }
}

void processCommand(WebSocket s, String command, Directory dir) {
  List splitCommand = command.split(' ');
  String executable = splitCommand[0];
  List args = (splitCommand.length > 1) ? splitCommand.getRange(1, splitCommand.length).toList() : [];
  Process.run(executable, args, workingDirectory: dir.path).then((ProcessResult results) {
    s.add('[[CONSOLE_COMMAND]]' + results.stdout);
  });
}