#!/usr/bin/env dart

import 'package:args/command_runner.dart';

import '../lib/server.dart';

void main(List<String> args) {
  const String version = '0.5.0 Alpha';

  CommandRunner runner = new CommandRunner("cmdr", "UpDroid Commander - Robotics Software IDE.")
    ..addCommand(new InfoCommand(version))
    ..addCommand(new CmdrGuiCommand());

    runner.run(args).catchError((error) {
      if (error is! UsageException) throw error;
      print(error);
      return;
    });
}
