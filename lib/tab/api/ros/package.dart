part of ros;

/// A representation of a ROS package (as of Indigo).
class Package {
  Workspace workspace;
  String name;

  /// Creates a [Package] object.
  Package(this.workspace, this.name);

  /// Creates the [Package] on the filesystem with optional [dependencies].
  ///
  /// Equivalent to running 'catkin_create_pkg'.
  Future<Package> create([List<String> dependencies]) {
    Completer completer = new Completer();
    Process.run('bash', ['-c', '. /opt/ros/indigo/setup.bash && catkin_create_pkg --rosdistro indigo $name'], workingDirectory: workspace.src.path, runInShell: true).then((result) {
      completer.complete(this);
    });
    return completer.future;
  }
}