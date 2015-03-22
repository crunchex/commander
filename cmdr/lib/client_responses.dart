library client_responses;

import 'dart:io';
import 'dart:convert';
import 'dart:async';

void sendErrorMessage(WebSocket s) {
  s.add('[[CONSOLE_OUTPUT]]' + "Invalid Command");
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