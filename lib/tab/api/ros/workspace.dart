part of ros;

/// A representation of a ROS catkin workspace (as of Indigo).
///
/// Implements [Directory] and exposes wrappers for functions that are
/// exclusive to ROS catkin workspaces.
class Workspace {
  /// Creates a directory object pointing to the current working directory.
  static Directory get current => Directory.current;

  static bool isWorkspace(String path) {
    return FileSystemEntity.isFileSync('$path/src/CMakeLists.txt');
  }

  final Directory _delegate;

  bool _building;

  /// Creates a [Workspace] object.
  ///
  /// If [path] is a relative path, it will be interpreted relative to the
  /// current working directory (see [Workspace.current]), when used.
  ///
  /// If [path] is an absolute path, it will be immune to changes to the
  /// current working directory.
  Workspace(String path) : _delegate = new Directory(path) {
    _building = false;
  }

  /// Returns the [Directory] for the default src directory.
  Directory get src {
    return new Directory('${_delegate.path}/src');
  }

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

  Future<List<String>> getContentsAsStrings() {
    Completer c = new Completer();
    List<String> files = [];
    _delegate.list(recursive: true, followLinks: false).listen((FileSystemEntity file) async {
      if (file.path.contains('$path/src')) {
        bool isFile = await FileSystemEntity.isFile(file.path);
        String fileString = isFile ? 'F:${file.path}' : 'D:${file.path}';
        files.add(fileString);
      }
    }).onDone(() => c.complete(files));
    return c.future;
  }

  Stream listContents() {
    return _delegate.list(recursive: true, followLinks: false).transform(toWorkspaceContents(path)).asBroadcastStream();
  }

  List<String> listContentsSync() {
    List<String> directories = [];
    List<String> files = [];

    List<FileSystemEntity> entityList = _delegate.listSync(recursive: true, followLinks: false);
    entityList.forEach((FileSystemEntity entity) {
      if (entity.path.contains('$path/src')) {
        String fileString = FileSystemEntity.isFileSync(entity.path) ? 'F:${entity.path}' : 'D:${entity.path}';

        fileString.startsWith('F:') ? files.add(fileString) : directories.add(fileString);
      }
    });

    // Alphabetize each.
    directories.sort();
    files.sort();

    // Append all files together after all the directories.
    directories.addAll(files);
    return directories;
  }

  Stream listLaunchers() {
    return src.list(recursive: true, followLinks: false).transform(toLaunchFiles()).asBroadcastStream();
  }

  /// Transformer to convert serialized [WebSocket] messages into the UpDroidMessage.
  StreamTransformer toWorkspaceContents(String path) => new StreamTransformer.fromHandlers(handleData: (file, sink) {
    if (file.path.contains('$path/src')) {
      FileSystemEntity.isFile(file.path).then((bool isFile) {
        String fileString = isFile ? 'F:${file.path}' : 'D:${file.path}';
        sink.add(fileString);
      });
    }
  });

  StreamTransformer toLaunchFiles() => new StreamTransformer.fromHandlers(handleData: (file, sink) {
    String filename = file.path.split('/').last;

    // Scan for launch files.
    if (filename.endsWith('.launch')) {
      // Extract args from launch file.
      String contents = file.readAsStringSync();
      XmlDocument xml = parse(contents);

      List args = [];
      XmlElement launchNode = xml.findElements('launch').first;
      List<XmlElement> argNodes = launchNode.findElements('arg');
      argNodes.forEach((XmlElement node) {
        List singleArg = new List(2);
        bool validArg = false;

        if (node.attributes.first.name.toString() == 'name') {
          singleArg[0] = node.attributes.first.value;
          validArg = true;
        }

        node.attributes.forEach((XmlAttribute attribute) {
          if (attribute.name.toString() == 'default') singleArg[1] = attribute.value;
        });

        // Only add an arg if the first attribute is the name.
        if (validArg) args.add(singleArg);
      });

      // Only pick up launch files that are within the 'launch' dir
      // at the top level of the package root.
      //print(f.parent.path.split('/').toString());
      if (file.parent.path.split('/').last == 'launch') {
        Directory package = file.parent.parent;

        sink.add({
          'package-path': package.path,
          'node': filename,
          'args': args
        });
      }
    }
  });

//  List<FileSystemEntity> listSync({bool recursive: false, bool followLinks: true}) {
//    return _delegate.listSync(recursive: recursive, followLinks: followLinks);
//  }

  /// Cleans the workspace by removing build, devel, and install directories.
  Future<ProcessResult> clean() => Process.run('rm', ['-rf', 'build', 'devel', 'install'], workingDirectory: path, runInShell: true);

  /// Builds the workspace.
  ///
  /// Equivalent to running 'catkin_make' and 'catkin_make install'.
  Future<ProcessResult> buildWorkspace() {
    Completer c = new Completer();
    if (_building) c.complete(null);

    _building = true;
    String buildCommand = '/opt/ros/indigo/setup.bash && catkin_make && catkin_make install';
    Process.run('bash', ['-c', '. $buildCommand'], workingDirectory: path, runInShell: true).then((ProcessResult result) {
      _building = false;
      c.complete(result);
    });

    return c.future;}

  /// Builds a package.
  ///
  /// Equivalent to running 'catkin_make --pkg' and 'catkin_make install'.
  Future<ProcessResult> buildPackage(String packageName) {
    Completer c = new Completer();
    if (_building) c.complete(null);

    _building = true;
    String buildCommand = '/opt/ros/indigo/setup.bash && catkin_make --pkg $packageName && catkin_make install';
    Process.run('bash', ['-c', '. $buildCommand'], workingDirectory: path, runInShell: true).then((ProcessResult result) {
      _building = false;
      c.complete(result);
    });

    return c.future;
  }

  /// Builds multiple packages given a list of package names..
  ///
  /// Equivalent to running 'catkin_make --pkg pkg1...pk2...' and 'catkin_make install'.
  Future<ProcessResult> buildPackages(List<String> packageNames) {
    Completer c = new Completer();
    if (_building) c.complete(null);

    _building = true;
    String packageListString = '';
    packageNames.forEach((String packageName) => packageListString += ' $packageName');
    String buildCommand = '/opt/ros/indigo/setup.bash && catkin_make --pkg$packageListString && catkin_make install';
    Process.run('bash', ['-c', '. $buildCommand'], workingDirectory: path, runInShell: true).then((ProcessResult result) {
      _building = false;
      c.complete(result);
    });

    return c.future;
  }

  Future<ProcessResult> createPackage(String name, List<String> dependencies) {
    String dependenciestring = '';

    if (dependencies.isNotEmpty) {
      dependencies.forEach((String dependency) => dependenciestring += ' $dependency');
    }

    String createPackageCommand = '$path/devel/setup.bash && catkin_create_pkg $name$dependenciestring';
    return Process.run('bash', ['-c', '. $createPackageCommand'], workingDirectory: '$path/src/', runInShell: true);
  }

  void runNode(String packageName, String nodeName, List args) {
    String launchArgs = '';
    args.forEach((List<String> arg) {
      String argString;
      if (arg[1].startsWith('\'') && arg[1].endsWith('\'')) {
        argString = arg[1];
      } else {
        argString = '\'${arg[1]}\'';
      }
      if (!arg[1].isEmpty) launchArgs += ' ${arg[0]}:=$argString';
    });

    String runCommand = '$path/devel/setup.bash && roscd $packageName && roslaunch $packageName $nodeName$launchArgs';
    help.debug('running: $runCommand', 0);
    Process.run('bash', ['-c', '. $runCommand'], runInShell: true).then((process) {
      // TODO: pipe the output somewhere.
//      stdout.addStream(process.stdout);
//      stderr.addStream(process.stderr);
    });
  }

  /// Gets the path of this workspace.
  String get path => _delegate.path;
}