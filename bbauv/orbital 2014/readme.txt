Vision GUI 
=======================
- Can run using alias: vision
- Can also run using: rosrun GUI GUI_node

Controls GUI 
=======================
- run using: rosrun GUI Control_node
or run using the alias: PID
- can modify the "live" parameter in command line as: rosrun GUI Control_node _live:=true
If live is false, parameters initialised to 0; otherwise, it will subscribe to the relevant topics
Default mode is live:=false
- Do need to fill in the subscribers name 
- Still need to fix actionlib with the movebaseclient :) 

Compiling Setting up QCustomPlot (already done for the project)
======================
- Download both the source code directory and the shared libraries directory from QCustomPlot website
- Compile the code according to the instruction in the shared libraries directory
- Add link_directories(libs) to the CMakeLists.txt of GUI package
- Add PrintSupport to the node's qt_use_module macro
- Copy qcustomplot.h to include directory
- Add "qcustomplot.h" in the any GUI code that uses it
- Add a generic Widget in QtDesigner and promote it to QCustomPlot class the header set as qcustomplot.h

Running Controls GUI with QCustomPlot
=======================
- QCustomPlot shared libraries are in the libs directory
- add export LD_LIBRARY_PATH="{path_to_the_libs_dir}":$LD_LIBRARY_PATH to ~/.bashrc
  or copy the libraries into typical shared libraries directories like usr/local/lib or usr/lib

