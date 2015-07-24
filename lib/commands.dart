part of cmdr;

class CmdrGuiCommand extends Command {
  final name = "gui";
  final description = "Launch UpDroid Commander in your browser.";

  CmdrGuiCommand() {
    argParser.addFlag('debug', abbr: 'd', defaultsTo: CmdrServer.defaultDebugFlag,
        help: 'Prints debug messages to /var/log/updroid/cmdr.log');
    argParser.addFlag('quiet', abbr: 'q', defaultsTo: CmdrServer.defaultQuiet,
        help: 'Starts cmdr with no output on stdout.');
    argParser.addOption('workspace', abbr: 'w', defaultsTo: CmdrServer.defaultUprootPath,
        help: 'Overrides the default workspaces directory for cmdr.');
    argParser.addOption('path', abbr: 'p', defaultsTo: CmdrServer.defaultInstallationPath,
        help: 'Overrides cmdr\'s default installation path.\nLocation of executables, static gui files, and misc.\nOnly useful for cmdr developers.');
  }

  void run() {
    help.enableDebug(argResults['debug']);
    new CmdrServer(argResults);
  }
}

class InfoCommand extends Command {
  final name = "info";
  final description = "Display general information about cmdr.";

  String version;

  InfoCommand(this.version) {
    argParser.addFlag('version', abbr: 'v', defaultsTo: false,
      help: 'Prints the version number.');
  }

  void run() {
    if (argResults['version']) print(version);
  }
}