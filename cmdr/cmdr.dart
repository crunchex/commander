#!/usr/bin/env dart

import 'package:args/args.dart';

import 'server.dart';
import 'lib/server_helper.dart' as help;

void main(List<String> args) {
  // Create an args parser.
  ArgParser parser = new ArgParser();
  parser.addFlag('debug', abbr: 'd', defaultsTo: UpDroidServer.defaultDebugFlag);

  // Add the 'gui' command and an option to override the default workspace.
  ArgParser command = parser.addCommand('gui');
  command.addOption('workspace', abbr: 'w', defaultsTo: UpDroidServer.defaultWorkspacePath);
  command.addOption('path', abbr: 'p', defaultsTo: UpDroidServer.defaultGuiPath);
  ArgResults results = parser.parse(args);

  // Set up logging.
  bool debugFlag = results['debug'];
  help.enableDebug(debugFlag);

  UpDroidServer server = new UpDroidServer(results);
}
