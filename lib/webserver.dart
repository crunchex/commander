part of cmdr;

class WebServer {
  ArgResults _args;
  CmdrMailbox _mailbox;
  Map<String, Map<int, PanelInterface>> _panels;
  Map<String, Map<int, TabInterface>> _tabs;
  Map<String, List<String>> _idQueue;

  WebServer(ArgResults args, CmdrMailbox mailbox, Map tabs, Map panels, Map idQueue) {
    _args = args;
    _mailbox = mailbox;
    _tabs = tabs;
    _panels = panels;
    _idQueue = idQueue;
  }

  /// Initializes and HTTP server to serve the gui and handle [WebSocket] requests.
  void init() {
    VirtualDirectory virDir = _getVirDir();

    // Set up an HTTP webserver and listen for standard page requests or upgraded
    // [WebSocket] requests.
    HttpServer.bind(InternetAddress.ANY_IP_V4, 12060).then((HttpServer server) {
      _printStartMessage();

      debug("HttpServer listening on port:${server.port}...", 0);
      server.asBroadcastStream()
      .listen((HttpRequest request) => _routeRequest(request, virDir))
      .asFuture()  // Automatically cancels on error.
      .catchError((_) => debug("caught error", 1));
    });
  }

  void _routeRequest(HttpRequest request, VirtualDirectory virDir) {
    // WebSocket requests are considered "upgraded" HTTP requests.
    if (!WebSocketTransformer.isUpgradeRequest(request)) {
      _handleStandardRequest(request, virDir);
      return;
    }

    debug('Upgraded request received: ${request.uri.path}', 0);

    // TODO: objectIDs start at 1, but List indexes start at 0 - fix this.
    int objectID = int.parse(request.uri.pathSegments[1]);
    String type = request.uri.pathSegments[0];

    if (type == Tab.upcomName) {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _mailbox.handleWebSocket(ws, request));
      return;
    }

    if (_tabs.containsKey(type)) {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _tabs[type][objectID].mailbox.receive(ws, request));
    } else if (_panels.containsKey(type)) {
      WebSocketTransformer.upgrade(request)
      .then((WebSocket ws) => _panels[type][objectID].mailbox.receive(ws, request));
    }
  }

  void _handleStandardRequest(HttpRequest request, VirtualDirectory virDir) {
    debug("${request.method} request for: ${request.uri.path}", 0);

    List<String> segs = request.uri.pathSegments;
    if (segs.length > 1 && segs[segs.length - 2] == 'requestId') {
      request.response
        ..headers.contentType = ContentType.JSON
        ..write(_idQueue[segs.last].first)
        ..close();

      // Pop the ID off the queue.
      _idQueue[segs.last].removeAt(0);
      return;
    }

    if (virDir != null) {
      virDir.serveRequest(request);
    } else {
      debug('ERROR: no Virtual Directory to serve', 1);
    }
  }

  /// Returns a [VirtualDirectory] set up with a path from [results].
  VirtualDirectory _getVirDir() {
    String guiPath = '${_args['path']}/web';
    VirtualDirectory virDir;
    virDir = new VirtualDirectory(Platform.script.resolve(guiPath).toFilePath())
      ..allowDirectoryListing = true
      ..followLinks = true
    // Uncomment to serve to Dartium for debugging.
    //..jailRoot = false
      ..directoryHandler = (dir, request) {
      // Redirects '/' to 'index.html'
      var indexUri = new Uri.file(dir.path).resolve('index.html');
      virDir.serveFile(new File(indexUri.toFilePath()), request);
    };

    return virDir;
  }

  void _printStartMessage() {
    if (_args['quiet']) return;

    print('[UpDroid Commander serving on port 12060]');
    print('You can now enter "localhost:12060" in your browser on this machine,');
    print('  or "${Process.runSync('hostname', ['-I']).stdout.replaceAll('\n', '').trim()}:12060" on a machine in the same network.');

    if (!Platform.isMacOS) {
      ProcessResult pkgStatus = Process.runSync('dpkg' , ['-s', 'libnss-mdns', '|', 'grep', 'Status']);
      if (pkgStatus.stdout.contains('install ok installed')) {
        print('  or "${Platform.localHostname}.local:12060" on a Bonjour/libnss-mdns equipped machine.');
      }
    }

    print('Ctrl-C to exit.');
  }
}