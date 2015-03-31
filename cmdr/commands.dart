part of updroid_server;

class CmdrGuiCommand extends Command {
  final name = "gui";
  final description = "Launch UpDroid Commander in your browser.";

  CmdrGuiCommand() {
    argParser.addFlag('debug', abbr: 'd', defaultsTo: CmdrServer.defaultDebugFlag, help: 'Prints debug messages to server.log');
    argParser.addOption('workspace', abbr: 'w', defaultsTo: CmdrServer.defaultWorkspacePath, help: 'Overrides the default workspace for Commander.');
    argParser.addOption('path', abbr: 'p', defaultsTo: CmdrServer.defaultGuiPath, help: 'Overrides the default path of the gui files location.');
  }

  void run() {
    help.enableDebug(argResults['debug']);
    CmdrServer server = new CmdrServer(argResults);
  }
}