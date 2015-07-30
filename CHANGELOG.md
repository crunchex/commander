# Change Log

## 0.5.2 - 2015-7-30

### New
- Camera tabs now display the current video device on tab handle hover.
- Added individual package cleaning via right-clicking on packages within the Explorer Workspace view.
- Package build output is now appended to a file at `/var/log/updroid/build.log`. cmdr will attempt to create this file if the directory permissions allow it to.
- Added a new top-level command to cmdr (see cmdr --help). Intended to be run within a Console tab to view package build output.

### Changed
- Explorer now automatically loads solo workspaces detected in `~/uproot`.
- Internal installation path structure has changed (in `/opt/updroid/cmdr`).
- The cmdr executable is now installed within `/opt/updroid/cmdr` with a symlink dropped into `/usr/bin`.
- Console's "dark" theme is now a little darker.
- Explorer's "build" buttons have been renamed "verify".
- Explorer's verify button will now build the entire workspace by default if no packages are selected.
- Improved cmdr crash handling, where the GUI page can be refreshed after dimissing a disconnect alert and restarting cmdr.

### Fixed
- Console should now properly resize automatically once the frontend and backend are both loaded.
- Fixed Explorer issue with building the top-level 
- Fixed Explorer issues with building ROS metapackages.
- Fixed a bug where closing a Camera tab would occasionally crash cmdr.

## 0.5.1 - 2015-7-16

### New
- Tab columns can now be "maximized" (where other Tab columns become hidden).
- Tabs can now be "cloned" (in terms of type) and moved between columns via a right-click menu from their handles.

### Changed
- Changed the icon for Launchers view button in the lower-right corner of Explorer.
- Small aesthetic improvements.

### Fixed
- Fixed a bug where a window alert would fire more than once after a permanent disconnection to the server-side.

## 0.5.0 - 2015-7-10

### New
- Redesigned Explorer, both visually and functionally.
- New look and color scheme (as always, a work in progress).

### Changed
- Replaced all drag-and-drop interaction with right-click and dropdown menus where appropriate.
- Removed basic integration with git (temporary until we can do it right)!
- Window.alert gets more usage where appropriate.
- A temporary help-button is now in the top-right corner.

### Fixed
- Numerous bug fixes... too many to count due to extensive rewrite.

## 0.4.8 - 2015-6-4

### New

- Close button added to the corner of tab handles.
- Added support for Dart 1.10.0 (x86_64 only).

### Changed

- Set max number of open tabs per column to 4 (temporary).
- Double-clicking a tab no longer replicates tabs.
- When a new Camera tab is opened, an available device is automatically selected.

### Fixed

- Fixed some graphical glitches when closing and switching tabs.
- Fixed text colors in certain areas changing when they're not supposed to.
- Fixed Ctrl-S on a file loaded into Editor crashing cmdr.

## 0.4.7 - 2015-5-28

### New

- Unsaved changes to a file loaded in Editor are now indicated via asterisk by the filename.
- A browser alert has been added that indicates when UpCom has been disconnected from the server side.
- Save-As in the Editor file menu now includes a checkbox to make a file executable.
- Added ROS Templates menu to Editor.

### Changed

- Switching to the Control tab in Explorer now automatically populates the Node/Launch file list.
- The Node/Launch file list now clears when switching out of the Control tab.
- Console cursor no longer blocks out text character beneath.
- General stablity and appearance improvements to Console.
- CMakeLists.txt is no longer filtered out of the Explorer view.

### Fixed

- Fixed broken numpad, delete key, arrow keys, single quotes, & key in Console.
- Fixed text highlighting in Console.
- Fixed resizing the window spamming Console resize messages on the browser's debug console.
- Fixed broken scrolling in some terminal programs when run in Console. For example, vim should work now.
- Fixed node titles/graphics not wrapping correctly in Explorer's Control tab.

## 0.4.6 - 2015-5-21

### Important

- Raspberry Pi users need to install ffmpeg >=2.6.2 before using UpCom

### New

- Added Ctrl+S hotkey for saving in editor.
- Added favicon.

### Changed

- Switching between Control and Explorer more clear.
- Improved stability for multiple workspaces.
- Minor UI tweaks.

### Fixed

- Fixed bug where saving a new file in editor without a selected path caused UpCom to crash.

## 0.4.5 - 2015-5-15

### Important

- Raspberry Pi support is still very early, so please consider the inherent limitations of the hardware and things that still need to be optimized.

### New

- Initial Raspberry Pi support (experimental!).
- Naming a workspace when a new one is added (via the Control dropdown button).
- Camera Tab now includes a Devices menu to select the video input source.

### Changed

- Replaced the Close Workspace button with a Delete Workspace button.
- Settings menu (with placeholder menu items) is gone from the Camera tab. But will reappear again in the future as fully functional.

### Fixed

- Fixed some freezing issues with live video streaming in the Camera Tab on the desktop version.
- Fixed a bug with Explorer picking up filesystem changes to closed workspaces.

## 0.4.4 - 2015-5-9

### Fixed

- Fixed List Nodes button crashing cmdr.

## 0.4.3 - 2015-5-9

### Important

- If cmdr needs to be restarted (e.g. after a crash), close UpCom in the browser before restarting it.

### New

- Add/Close Workspace buttons have been implemented.
- If `/home/uproot` is empty, a default workspace will be created and initialized for you. Note, it is not yet possible to rename workspaces through UpCom.

### Changed


### Fixed

- Fixed blank dummy file being displayed in Explorer in an empty workspace.
- Fixed Console tab throwing out disconnect error messages.
- Improved general reliability of closing tabs.
- Improved UpCom layout with responsive height.
- Fixed some graphical glitches involving borders.

## 0.4.2 - 2015-5-7

### New

- Explorer handles multiple workspaces contained in `/home/uproot`
- cmdr displays minimalist feedback once run from CLI.
- CLI command to cmdr called 'info'.

### Changed

- `/home/uproot` is now a directory for workspaces, rather than a workspace itself.
- Available nodes and launch files are displayed in the Control portion of the Explorer, rather than in a popup.
- The Run button runs the currently selected node, rather than displaying the available node list.

### Fixed

- Fixed Top level dropzone highlighting when a draggable file is already at the top level.
- Fixed renaming a file not updating the displayed filename in Editor.
- Fixed Editor not highlighting in certain cases where files are dragged from Explorer.
- Fixed Editor not scrolling to the top when a new file is opened.
- Fixed some graphical glitches involving borders.

## 0.4.1 - 2015-4-30

### New

- Added progress indicator to global control buttons.
- Added a reminder to use UpCom's feedback button.
- Added a 'Clean' workspace global control button.

### Changed

- Improved functionality for existing global control buttons.
- Improved layout for 'Run' button pop-up menu.
- 'Run' button menu now supports launch files with arguments.

### Fixed

- Fixed Console clipboard paste via right-click.
- Fixed Console arrow keys in normal mode.