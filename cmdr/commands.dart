part of updroid_server;

class CmdrGuiCommand extends Command {
  final name = "gui";
  final description = "Launch UpDroid Commander in your browser.";

  CmdrGuiCommand() {
    argParser.addFlag('debug', abbr: 'd', defaultsTo: CmdrServer.defaultDebugFlag,
        help: 'Prints debug messages to cmdr.log');
    argParser.addOption('workspace', abbr: 'w', defaultsTo: CmdrServer.defaultWorkspacePath,
        help: 'Overrides the default workspace for cmdr.');
    argParser.addOption('path', abbr: 'p', defaultsTo: CmdrServer.defaultGuiPath,
        help: 'Overrides the default path of the gui files location.\nOnly useful for cmdr developers.');
  }

  void run() {
    help.enableDebug(argResults['debug']);
    new CmdrServer(argResults);
  }
}