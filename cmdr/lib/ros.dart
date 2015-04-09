library ros;

import 'dart:io';
import 'dart:async';
import 'dart:convert';

import 'server_helper.dart' as help;

abstract class Ros {
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
                String execName =
                    line.substring(line.indexOf('(') + 1, line.indexOf(' '));
                nodeList.add({
                  'package': package.path.split('/').last,
                  'node': execName
                });
              }
            });
          }

          // Scan for Python nodes with a main function.
          if (filename.contains('.py')) {
            List<String> contents = f.readAsLinesSync();
            contents.forEach((line) {
              if (line.contains('rospy.init_node') && !line.contains('#')) {
                String execName = line.substring(line.indexOf('\'') + 1);
                execName = execName.substring(0, execName.indexOf('\'')) + '.py';

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
                nodeList.add({
                  'package': package.path.split('/').last,
                  'node': execName
                });
              }
            });
          }
        }
      });

      ws.add('[[CATKIN_NODE_LIST]]' + JSON.encode(nodeList));
    });
  }

  static void runNode(String package, String node) {
    Process.run('pgrep', ['roscore']).then((result) {
      if (result.stdout == '') {
        Process
            .start('roscore', [], mode: ProcessStartMode.DETACHED)
            .then((result) {
          Process.run('rosrun', [package, node]).then((result) {
            help.debug(result.stdout, 0);
            help.debug(result.stderr, 0);
          });
        });
      } else {
        Process.run('rosrun', [package, node]).then((result) {
          help.debug(result.stdout, 0);
          help.debug(result.stderr, 0);
        });
      }
    });
  }
}