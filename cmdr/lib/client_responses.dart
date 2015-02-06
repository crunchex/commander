library client_responses;

import 'dart:io';
import 'dart:convert';
import 'dart:async';
import 'server_helper.dart' as help;

void sendDirectory(WebSocket s, Directory dir) {
  help.getDirectory(dir).then((files) {
    s.add('[[EXPLORER_DIRECTORY_LIST]]' + files.toString());
  });
}

void refreshDirectory(WebSocket s, Directory dir) {
  help.getDirectory(dir).then((files) {
    s.add('[[EXPLORER_DIRECTORY_REFRESH]]' + files.toString());
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

void requestFilename(WebSocket s, String path) {
  String filename = 'untitled.py';
  File newFile = new File(path + '/' + filename);
  
  int untitledNum = 0;
  while (newFile.existsSync()) {
    untitledNum++;
    filename = 'untitled' + untitledNum.toString() + '.py';
    newFile = new File(path + '/' + filename); 
  }
  
  s.add('[[EDITOR_NEW_FILENAME]]' + filename);
}

void fsNewFile(String path) {
  String fullPath = path + '/untitled.py';
  File newFile = new File(fullPath);
  
  int untitledNum = 0;
  while (newFile.existsSync()) {
    untitledNum++;
    fullPath = path + '/untitled' + untitledNum.toString() + '.py';
    newFile = new File(fullPath); 
  }

  newFile.create();
}

void fsNewFolder(String path) {
  String fullPath = path;
  Directory newFolder = new Directory(fullPath);
  
  int untitledNum = 0;
  while(newFolder.existsSync()) {
    untitledNum++;
    fullPath = path + untitledNum.toString();
    newFolder = new Directory(fullPath);
  }
  
  newFolder.createSync();
}

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

void processCommand(WebSocket s, StreamController<String> inputStream, String command, Directory dir) {
  List splitCommand = command.split(' ');
  String executable = splitCommand[0];
  List args = (splitCommand.length > 1) ? splitCommand.getRange(1, splitCommand.length).toList() : [];

  Process.start('/usr/bin/stdbuf', ["--output=L", executable], workingDirectory: dir.path).then((Process process) {
    process.stdout
      .transform(UTF8.decoder)
      .listen((data) {
        s.add('[[CONSOLE_OUTPUT]]' + data);
      });
    process.stdin.addStream(inputStream.stream.transform(UTF8.encoder));
    process.exitCode.then((exitCode) => s.add('[[CONSOLE_EXIT]]' + exitCode.toString()));
  });
}

void passInput(StreamController<String> inputStream, String input) {
  inputStream.add(input);
}