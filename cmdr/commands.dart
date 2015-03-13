part of updroid_server;

class GuiCommand extends Command {
  final name = "gui";
  final description = "Launch UpDroid Commander in your browser.";

  GuiCommand() {
    argParser.addFlag('debug', abbr: 'd', defaultsTo: UpDroidServer.defaultDebugFlag, help: 'Prints debug messages to server.log');
    argParser.addFlag('serveronly', abbr: 's', defaultsTo: false, help: 'Disables gui-serving (assumes DartEditor used).');
    argParser.addOption('workspace', abbr: 'w', defaultsTo: UpDroidServer.defaultWorkspacePath, help: 'Overrides the default workspace for Commander.');
    argParser.addOption('path', abbr: 'p', defaultsTo: UpDroidServer.defaultGuiPath, help: 'Overrides the default path of the gui files location.');
  }

  void run() {
    help.enableDebug(argResults['debug']);
    UpDroidServer server = new UpDroidServer(argResults);
  }
}