library ros;

import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'package:path/path.dart' as pathLib;

import 'package:xml/xml.dart';

import '../server_helper.dart' as help;

part 'workspace.dart';

abstract class Ros {
  static void nodeList(Directory workspace, WebSocket ws) {
    List launchList = [];
    List nodeList = [];

    Directory src = new Directory(pathLib.normalize(workspace.path + '/src'));

    help.getDirectory(src).then((fsEntities) {
      Directory package;
      fsEntities.forEach((f) {
        if (FileSystemEntity.isFileSync(f.path)) {
          String filename = f.path.split('/').last;

          // Scan for launch files.
          if (filename.contains('.launch')) {
            // Extract args from launch file.
            String contents = f.readAsStringSync();
            XmlDocument xml = parse(contents);

            List args = [];
            List<XmlElement> argNodes = xml.findAllElements('arg');
            argNodes.forEach((XmlElement node) {
              String xmlString = node.toXmlString();

              // Extract name.
              String namePart = xmlString.replaceAll(new RegExp(r'<[a-z ]+name="'), '');
              String name = namePart.substring(0, namePart.indexOf('"'));

              // Extract default and return both, or just return the name.
              if (xmlString.contains('default="')) {
                String defaultPart = xmlString.replaceAll(new RegExp(r'<[a-z ="]+default="'), '');
                String defaultString = defaultPart.substring(0, defaultPart.indexOf('"'));
                args.add([name, defaultString]);
              } else {
                args.add([name]);
              }
            });

            // Only pick up launch files that are within the 'launch' dir
            // at the top level of the package root.
            //print(f.parent.path.split('/').toString());
            if (f.parent.path.split('/').last == 'launch') {
              package = f.parent.parent;

              launchList.add({
                'package': package.path.split('/').last,
                'package-path': package.path,
                'node': filename,
                'args': args
              });
            }
          }
//
//          if (filename == 'CMakeLists.txt') {
//            // Get the parent package name.
//            package = f.parent;
//
//            // Read CMakeLists.txt for C++ executables.
//            List<String> contents = f.readAsLinesSync();
//            contents.forEach((line) {
//              if (line.contains('add_executable') && !line.contains('#')) {
//                String execName =
//                    line.substring(line.indexOf('(') + 1, line.indexOf(' '));
//                nodeList.add({
//                  'package': package.path.split('/').last,
//                  'node': execName
//                });
//              }
//            });
//          }
//
//          // Scan for Python nodes with a main function.
//          if (filename.contains('.py')) {
//            List<String> contents = f.readAsLinesSync();
//            contents.forEach((line) {
//              if (line.contains('rospy.init_node') && !line.contains('#')) {
//                String execName = line.substring(line.indexOf('\'') + 1);
//                execName = execName.substring(0, execName.indexOf('\'')) + '.py';
//
//                // Get the parent package name.
//                Directory parent = f.parent;
//                while (true) {
//                  String parentName = parent.path.split('/').last;
//                  if (parentName != 'src' && parentName != 'scripts') {
//                    package = parent;
//                    break;
//                  }
//                  parent = parent.parent;
//                }
//                nodeList.add({
//                  'package': package.path.split('/').last,
//                  'node': execName
//                });
//              }
//            });
//          }
        }
      });

      launchList.addAll(nodeList);
      ws.add('[[CATKIN_NODE_LIST]]' + JSON.encode(launchList));
    });
  }

  static void runNode(Workspace ws, String nodeInfo) {
    List nodeMap = JSON.decode(nodeInfo);

    if (nodeMap[2].contains('.launch')) {
      String rosLaunch = '${nodeMap[0]} ${nodeMap[2]}';
      if (nodeMap.length > 3) rosLaunch += ' ${nodeMap[3]}';
      Process.start('bash', ['-c', '. ${ws.path}/devel/setup.bash && roscd ${nodeMap[0]} && roslaunch $rosLaunch'], runInShell: true).then((process) {
        stdout.addStream(process.stdout);
        stderr.addStream(process.stderr);
      });
    } else {
      Process.run('pgrep', ['roscore']).then((result) {
        if (result.stdout == '') {
          Process
              .start('roscore', [], mode: ProcessStartMode.DETACHED)
              .then((result) {
                  Process.run('rosrun', [nodeMap[0], nodeMap[1]]).then((result) {
                    help.debug(result.stdout, 0);
                    help.debug(result.stderr, 0);
                  });
                });
        } else {
          Process.run('rosrun', [nodeMap[0], nodeMap[1]]).then((result) {
            help.debug(result.stdout, 0);
            help.debug(result.stderr, 0);
          });
        }
      });
    }
  }
}