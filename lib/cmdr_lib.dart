part of cmdr;

StreamTransformer pluginInfoExtractor(String type) {
  return new StreamTransformer.fromHandlers(handleData: (event, sink) {
    File pluginInfoJson = new File(pathLib.normalize('${event.path}/${type}info.json'));
    String pluginInfoString = pluginInfoJson.readAsStringSync();
    sink.add(JSON.decode(pluginInfoString));
  });
}

Future<Map> getPluginsInfo(String type, installationPath) {
  Completer c = new Completer();
  Map pluginsInfo = {};
  new Directory('$installationPath/bin/${type}s')
  .list()
  .transform(pluginInfoExtractor(type))
  .listen((Map panelInfoMap) => pluginsInfo[panelInfoMap['refName']] = panelInfoMap)
  .onDone(() => c.complete(pluginsInfo));
  return c.future;
}
