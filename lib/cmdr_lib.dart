part of cmdr;

StreamTransformer pluginInfoExtractor(PluginType type) {
  String typeString;
  if (type == PluginType.TAB) {
    typeString = 'tab';
  } else if (type == PluginType.PANEL) {
    typeString = 'panel';
  }

  return new StreamTransformer.fromHandlers(handleData: (event, sink) {
    File pluginInfoJson = new File(pathLib.normalize('${event.path}/${typeString}info.json'));
    String pluginInfoString = pluginInfoJson.readAsStringSync();
    sink.add(JSON.decode(pluginInfoString));
  });
}

Future<Map> getPluginsInfo(PluginType type, installationPath) {
  Completer c = new Completer();

  String typeString;
  if (type == PluginType.TAB) {
    typeString = 'tab';
  } else if (type == PluginType.PANEL) {
    typeString = 'panel';
  }

  Map pluginsInfo = {};
  new Directory('$installationPath/bin/${typeString}s')
  .list()
  .transform(pluginInfoExtractor(type))
  .listen((Map panelInfoMap) => pluginsInfo[panelInfoMap['refName']] = panelInfoMap)
  .onDone(() => c.complete(pluginsInfo));
  return c.future;
}
