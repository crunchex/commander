part of updroid_server;

class UpDroidPty {
  int ptyNum = 1;

  UpDroidPty(this.ptyNum, String workspacePath) {
    // TODO: this should be dynamically assigned when
    // multiple consoles are spawned.

    help.debug('Spawning UpDroidPty ($ptyNum)', 0);

    // Process launches 'cmdr-pty', a go program that provides a direct hook to a system pty.
    // See http://bitbucket.org/updroid/cmdr-pty
    Process.start('cmdr-pty', ['-addr', ':1206$ptyNum', '-size', '30x57'], environment: {'TERM':'vt100'}, workingDirectory: workspacePath).then((Process shell) {
      shell.stdout.listen((data) => help.debug('pty[$ptyNum] stdout: ${UTF8.decode(data)}', 0));
      shell.stderr.listen((data) => help.debug('pty[$ptyNum] stderr: ${UTF8.decode(data)}', 0));
    }).catchError((error) {
      if (error is! ProcessException) throw error;
      help.debug('cmdr-pty [$ptyNum]: run failed. Probably not installed', 1);
      return;
    });
  }
}