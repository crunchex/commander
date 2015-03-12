#!/usr/bin/env dart

import 'package:args/command_runner.dart';

import 'commands.dart';

void main(List<String> args) {
  CommandRunner runner = new CommandRunner("cmdr", "The UpDroid Command tool.")
    ..addCommand(new GuiCommand());

    runner.run(args).catchError((error) {
      if (error is! UsageException) throw error;
      print(error);
      return;
    });

}
