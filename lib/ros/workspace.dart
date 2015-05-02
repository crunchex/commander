part of ros;

/// A representation of a ROS catkin workspace (as of Indigo).
///
/// Implements [Directory] and exposes wrappers for functions that are
/// exclusive to ROS catkin workspaces.
class Workspace implements Directory {
  /// Creates a directory object pointing to the current working directory.
  static Directory get current => Directory.current;

  final Directory _delegate;

  /// Creates a [Workspace] object.
  ///
  /// If [path] is a relative path, it will be interpreted relative to the
  /// current working directory (see [Workspace.current]), when used.
  ///
  /// If [path] is an absolute path, it will be immune to changes to the
  /// current working directory.
  Workspace(String path) : _delegate = new Directory(path);

  /// Returns the [Directory] for the top-level src directory.
  Directory get src => new Directory('$path/src');

  /// Creates a [Workspace] with this name and a src directory within.
  ///
  /// If [recursive] is false, only the last directory in the path is
  /// created. If [recursive] is true, all non-existing path components
  /// are created. If the workspace already exists nothing is done.
  Future<Workspace> create({bool recursive: false}) {
    Completer completer = new Completer();
    _delegate.create(recursive: recursive).then((dir) {
      src.create().then((src) {
        completer.complete(this);
      });
    });

    return completer.future;
  }

  /// Sources the default ROS setup.bash file and initializes the workspace by
  /// calling 'catkin_init_workspace'.
  void initSync() {
    Process.runSync('bash', ['-c', '. /opt/ros/indigo/setup.bash && catkin_init_workspace'], workingDirectory: src.path, runInShell: true);
  }

  /// Cleans the workspace by removing build, devel, and install directories.
  Future<ProcessResult> clean() => Process.run('rm', ['-rf', 'build', 'devel', 'install'], workingDirectory: path, runInShell: true);

  /// Builds the workspace.
  ///
  /// Equivalent to running 'catkin_make' and 'catkin_make install'.
  Future<ProcessResult> build() {
    return Process.run('bash', ['-c', '. /opt/ros/indigo/setup.bash && catkin_make && catkin_make install'], workingDirectory: path, runInShell: true);
  }

  /// Gets the path of this workspace.
  String get path => _delegate.path;
}