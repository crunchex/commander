library git;

import 'dart:io';
import 'dart:convert';

abstract class Git {
  static void push(String dirPath, String password) {
    Process.run('git', ['commit', '-am', 'Pushed from UpDroid Commander.'], workingDirectory: dirPath).then((result) {
      Process.start('git', ['push'], workingDirectory: dirPath).then((process) {
        process.stderr.transform(UTF8.decoder).listen((String data) {
          if (data.contains('Password for')) {
            process.stdin.add(UTF8.encode(password));
          }
        });
      });
    });
  }
}