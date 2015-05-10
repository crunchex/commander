# Change Log

## 0.4.3 - 2015-5-9

### Important

	- If cmdr needs to be restarted (e.g. after a crash), close UpCom in the browser before restarting it.

### New

	- Add/Close Workspace buttons have been implemented.
	- If /home/uproot is empty, a default workspace will be created and initialized for you. Note, it is not yet possible to rename workspaces through UpCom.

### Changed


### Fixed

	- Fixed blank dummy file being displayed in Explorer in an empty workspace.
	- Fixed Console tab throwing out disconnect error messages.
	- Improved general reliability of closing tabs.
	- Improved UpCom layout with responsive height.
	- Fixed some graphical glitches involving borders.

## 0.4.2 - 2015-5-7

### New

	- Explorer handles multiple workspaces contained in /home/uproot
	- cmdr displays minimalist feedback once run from CLI.
	- CLI command to cmdr called 'info'.

### Changed

	- /home/uproot is now a directory for workspaces, rather than a workspace itself.
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