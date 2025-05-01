^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_polygon_selection_tool
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* Add separate executor and callback group (`#9 <https://github.com/marip8/rviz_polygon_selection_tool/issues/9>`_)
  * Updated tool to have its own executor such that service clients in the Rviz node (e.g., in other panels, tools) can call its service
  * Updated to support rolling
* Add Jazzy CI job (`#10 <https://github.com/marip8/rviz_polygon_selection_tool/issues/10>`_)
* Bug fix for Point Generation Gap to work while Lasso Mode is not activated (`#8 <https://github.com/marip8/rviz_polygon_selection_tool/issues/8>`_)
* Contributors: BryanMqz, Michael Ripperger

1.0.0 (2024-05-24)
------------------
* Multiple polygons rviz selection tool (`#7 <https://github.com/marip8/rviz_polygon_selection_tool/issues/7>`_)
  * Updated tool and service interface to support the selection of multiple polygons
  ---------
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Contributors: BryanMqz

0.1.1 (2024-05-24)
------------------
* Merge pull request `#4 <https://github.com/marip8/rviz_polygon_selection_tool/issues/4>`_ from marip8/update/deps
  Updated dependencies export
* Updated dependencies export
* Merge pull request `#3 <https://github.com/marip8/rviz_polygon_selection_tool/issues/3>`_ from marip8/update/loop-closure
  Various updates
* Updated README; added gif
* Simplified setting of colors
* Added feature to toggle loop closure
* Merge pull request `#2 <https://github.com/marip8/rviz_polygon_selection_tool/issues/2>`_ from marip8/feature/colors
  Add tool properties
* Change default point color to black
* Clang format
* Minor updates
* Added point size and coloring and line coloring
* Merge pull request `#1 <https://github.com/marip8/rviz_polygon_selection_tool/issues/1>`_ from marip8/feature/polygon-selection-tool
  Polygon Selection Tool
* Update package.xml
* Added CI files
* Add checks around include of CMake macro
* Added lasso mode
* Added formatting files and scripts
* Initial commit of Rviz polygon selection tool
* Initial commit
* Contributors: Michael Ripperger, Tyler Marr
