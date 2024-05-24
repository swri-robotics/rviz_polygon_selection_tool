# Rviz Polygon Selection Tool
Rviz tool plugin for creating polygon selections

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](https://github.com/marip8/rviz_polygon_selection_tool/actions/workflows/main.yml/badge.svg)](https://github.com/marip8/rviz_polygon_selection_tool/actions/workflows/main.yml)

## Operation

![example](docs/example.gif)

- **Left mouse button click**: Add point to current polygon
- **Middle mouse button click**: Erase current polygon selection
- **Right mouse button click**: Finish current polygon and start new polygon
- **Shortcut `delete`, `backspace`**: Remove all polygons

## Modes
- **Lasso**: click and hold left mouse button while drawing a shape
- **Click**: click and release the left mouse button to drop a vertex in the polygon

Toggle betweeen the modes by selecting the `Lasso mode` check box in the tool properties panel
