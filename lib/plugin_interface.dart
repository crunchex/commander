part of cmdr;

class PluginInterface {
  String refName;
  int id;
  Directory dir;
  List extra;

  Isolate plugin;
  IsolateMailbox mailbox;

  PluginInterface(PluginType type, String binPath, this.refName, this.id, this.dir, [this.extra]) {
    mailbox = new IsolateMailbox(refName, id);

    String pluginPath;
    if (type == PluginType.TAB) {
      pluginPath = '$binPath/tabs/$refName';
    } else if (type == PluginType.PANEL) {
      pluginPath = '$binPath/panels/$refName';
    }

    _spawnPlugin(pluginPath, new Uri.file(pathLib.normalize('$pluginPath/main.dart')));
  }

  Future _spawnPlugin(String pluginPath, Uri pluginFile) async {
    SendPort initialSendPort = mailbox.receivePort.sendPort;

    // Prepare the args.
    List args = [pluginPath, id, dir.path];
    if (extra != null) args.addAll(extra);

    plugin = await Isolate.spawnUri(pluginFile, args, initialSendPort);

    await for (var received in mailbox.receivePort) {
      if (mailbox.sendPort == null) {
        mailbox.sendPort = received;

        continue;
      }

      // At this point we are assuming messages are always strings.
      String message = received;
      if (message.startsWith('s:')) {
        mailbox.relay(new ServerMessage.fromString(message));
      } else if (message.startsWith('c:')) {
        mailbox.registerEndpoint(message);
      } else {
        Msg msg = new Msg.fromString(received);
        if (mailbox.endpointRegistry.contains(msg.header)) {
          mailbox.sendFromEndpoint(msg.body);
        } else {
          mailbox.send(new Msg.fromString(received));
        }
      }
    }
  }

  void close([int priority=Isolate.BEFORE_NEXT_EVENT]) {
    if (plugin == null) return;

    mailbox.close();
    plugin.kill(priority);
  }
}