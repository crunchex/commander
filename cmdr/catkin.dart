library catkin;

import 'dart:io';
import 'dart:async';

abstract class Catkin {
  static Future<String> buildWorkspace(String dirPath) {
    Completer completer = new Completer();

    String resultString = '';

    ProcessResult result = Process.runSync('catkin_make', [], workingDirectory: dirPath, runInShell: true);
    if (result.exitCode != 0) {
      //resultString += 'catkin_make: run failed - probably not sourced.';
      resultString += result.stderr;
    }

    completer.complete(resultString);
    return completer.future;
  }
}