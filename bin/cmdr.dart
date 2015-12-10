#!/usr/bin/env dart

import 'package:args/command_runner.dart';

import '../lib/server.dart';

void main(List<String> args) {
  const String version = '0.5.3 Alpha';

  // Entry point for [CmdrServer] can be found in commands.dart under [CmdrGuiCommand].
  CommandRunner runner = new CommandRunner("cmdr", "UpDroid Commander - Robotics Software IDE.")
    ..addCommand(new InfoCommand(version))
    ..addCommand(new BuildLogCommand())
    ..addCommand(new CmdrGuiCommand());

    runner.run(args).catchError((error) {
      if (error is! UsageException) throw error;
      print(error);
      return;
    });
}
