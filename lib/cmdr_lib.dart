part of cmdr;

StreamTransformer pluginInfoExtractor([PluginType type]) {
  String typeString;
  switch (type) {
    case PluginType.TAB:
      typeString = 'tab';
      break;
    case PluginType.PANEL:
      typeString = 'panel';
      break;
    default:
      typeString = '';
      break;
  }

  return new StreamTransformer.fromHandlers(handleData: (event, sink) {
    File pluginInfoJson = new File(pathLib.normalize('${event.path}/plugin.json'));
    String pluginInfoString = pluginInfoJson.readAsStringSync();

    Map<String, dynamic> pluginInfo = JSON.decode(pluginInfoString);
    if (pluginInfo['types'].contains(typeString)) {
      sink.add(JSON.decode(pluginInfoString));
    }
  });
}

Future<Map> getPluginsInfo(installationPath, [PluginType type]) {
  Completer c = new Completer();

  Map pluginsInfo = {};
  new Directory('$installationPath/bin/plugins')
  .list()
  .transform(pluginInfoExtractor(type))
  .listen((Map panelInfoMap) => pluginsInfo[panelInfoMap['refName']] = panelInfoMap)
  .onDone(() => c.complete(pluginsInfo));

  return c.future;
}
