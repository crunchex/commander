part of updroid_server;

class UpDroidPty {
  int ptyNum = 1;

  UpDroidPty(this.ptyNum, String workspacePath) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/crunchex/cmdr-pty
    Process.start('cmdr-pty', ['-addr', ':1206$ptyNum'], workingDirectory: workspacePath).then((Process shell) {
      shell.stdout.listen((data) => help.debug('pty[$ptyNum] stdout: ${UTF8.decode(data)}', 0));
      shell.stderr.listen((data) => help.debug('pty[$ptyNum] stderr: ${UTF8.decode(data)}', 0));
    });
  }
}