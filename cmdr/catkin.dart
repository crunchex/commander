library catkin;

import 'dart:io';
import 'dart:async';
import 'dart:convert';

import 'lib/server_helper.dart' as help;

abstract class Catkin {
  static Future<String> buildWorkspace(String dirPath) {
    Completer completer = new Completer();

    String resultString = '';

    ProcessResult result = Process.runSync('catkin_make', [],
        workingDirectory: dirPath, runInShell: true);
    if (result.exitCode != 0) {
      //resultString += 'catkin_make: run failed - probably not sourced.';
      resultString += result.stderr;
    }

    completer.complete(resultString);
    return completer.future;
  }

  static void nodeList(Directory workspace, WebSocket ws) {
    List nodeList = [];

    Directory src = new Directory('${workspace.path}/src');

    // Scan for Python nodes with a main function.
    help.getDirectory(src).then((fsEntities) {
      Directory package;
      fsEntities.forEach((f) {
        if (FileSystemEntity.isFileSync(f.path)) {
          String filename = f.path.split('/').last;

          if (filename == 'CMakeLists.txt') {
            // Get the parent package name.
            package = f.parent;

            // Read CMakeLists.txt for C++ executables.
            List<String> contents = f.readAsLinesSync();
            contents.forEach((line) {
              if (line.contains('add_executable') && !line.contains('#')) {
                String execName = line.substring(line.indexOf('(') + 1, line.indexOf(' '));
                nodeList.add({'package': package.path.split('/').last, 'node': execName});
              }
            });
          }

          // Scan for Python nodes with a main function.
          if (filename.contains('.py')) {
            String contents = f.readAsStringSync();
            if (contents.contains('__main__')) {
              // Get the parent package name.
              Directory parent = f.parent;
              while (true) {
                String parentName = parent.path.split('/').last;
                if (parentName != 'src' && parentName != 'scripts') {
                  package = parent;
                  break;
                }
                parent = parent.parent;
              }

              nodeList.add({'package': package.path.split('/').last, 'node': filename});
            }
          }
        }
      });

      ws.add('[[CATKIN_NODE_LIST]]' + JSON.encode(nodeList));
    });
  }

  static void runNode(String package, String node) {
    Process.run('pgrep', ['roscore'], runInShell: true).then((result) {
      if (result.stdout == '') {
        _runRosCore().then((result) {
          Process.run('rosrun', [package, node], runInShell: true);
        });
      } else {
        Process.run('rosrun', [package, node], runInShell: true);
      }
    });
  }

  static Future<ProcessResult> _runRosCore() {
    return Process.run('roscore', [], runInShell: true);
  }
}
