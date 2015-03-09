part of updroid_server;

class UpDroidPty {
  int _ptyNum;

  UpDroidPty(String workspacePath) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.
    _ptyNum = 0;

    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/crunchex/cmdr-pty
    Process.start('cmdr-pty', [], workingDirectory: workspacePath).then((Process shell) {
      shell.stdout.listen((data) => help.debug('pty[$_ptyNum] stdout: ${UTF8.decode(data)}', 0));
      shell.stderr.listen((data) => help.debug('pty[$_ptyNum] stderr: ${UTF8.decode(data)}', 0));
    });
  }
}