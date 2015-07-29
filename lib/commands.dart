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
    enableDebug(argResults['debug']);
    new CmdrServer(argResults);
  }
}

class BuildLogCommand extends Command {
  final name = "buildlog";
  final description = "Shortcut to view logged output from package builds.";

  BuildLogCommand();

  void run() {
    String logFileDir = '/var/log/updroid';
    File logFile = new File('$logFileDir/build.log');
    try {
      logFile.createSync(recursive:true);
    } on FileSystemException {
      print('The BuildLog command requires write access to $logFileDir.');
      print('Here\'s one way to enable (only need to do once):');
      print('  \$ sudo groupadd var-updroid');
      print('  \$ sudo usermod -a -G var-updroid ${Platform.environment['USER']}');
      print('  \$ sudo mkdir -p $logFileDir');
      print('  \$ sudo chown -R root:var-updroid $logFileDir');
      print('  \$ sudo chmod 2775 $logFileDir');
      print('Log out and back in (or restart session) for changes to take effect.');
      exit(2);
    }

    String runCommand = 'tail -f ${logFile.path}';
    Process.start('bash', ['-c', '$runCommand'], runInShell: true, environment: {'TERM':'vt100'}).then((shell) {
      shell.stdout.transform(UTF8.decoder).listen((data) => print(data));
      shell.stderr.transform(UTF8.decoder).listen((data) => print(data));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      debug('Unable to tail $logFileDir', 1);
      return;
    });
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