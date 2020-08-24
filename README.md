# batotp
A bisection algorithm for time-optimal trajectory planning along fully specified paths


1. Description --------------------------------------------------

This repository contains C++ source code for BA, a bisection algorithm for time-optimal trajectory planning along fully specified paths. A detailed description of the algorithm is provided in the following article:

Barnett, E. and Gosselin, C. "A Bisection Algorithm For Time-Optimal Trajectory Planning Along Fully Specified Paths", 2020, IEEE Transactions on Robotics,  DOI: 10.1109/TRO.2020.3010632, https://ieeexplore.ieee.org/document/9160859

Please cite this article when referring to the algorithm.

The source code is divided into two main subprojects:
- batotp: a static library that contains all of the functions used in the BA algorithm
- test: builds the execuatble batest, which depends on the batotp library. batest executes the main steps needed for a typical time-optimal trajectory planning problem: read in input data; interpolate input data; perform time optimization; interpolate output data; write output data.

Many configuration parameters can be set by using the input/config.dat file. Additional customization can be accomplished by modifying the batest subproject (test/main.cpp). For example, output data files are not needed for all applications. Finally, the BA algorithm can be readily integrated into other projects by linking the batotp static library.

2. License --------------------------------------------------

The source code for BA is subject to the terms of the Berkeley Software Distribution (BSD) 3-clause license. A copy of the BSD license is constined in the file COPYING.BSD. If a copy of the BSD license was not distributed with this file, you can obtain one at
https://opensource.org/licenses/BSD-3-Clause.

3. Compiling BA --------------------------------------------------

BA has been written to be compatible with the C++11 standard. Two commonly used compilers that are compatible with this standard are:
- gcc, version 4.8.1 or later
- Microsoft Visual Studio 2013 or later

The repository has been set up to compile with minimal configuration on several typical Windows and Linux installations. Specific instructions for a few installations are provided below. The fastest and easiest way to compile BA is by using only the gcc compiler in Linux or Windows. However, using Qt, Microsoft Visual Studio, or another IDE is the preferred method for integrating the BA code with other projects.

The only external dependency for BA is Eigen, a C++ template library for linear algebra:
http://eigen.tuxfamily.org

Two precompiled binaries are included in the Release folder. These can be used directly (skip ahead to 4. below) for compatible systems:
- bin/batest     for 64-bit Linux   (compiled in Ubuntu 16.04)
- bin/x64/batest.exe for 64-bit Windows (compiled in Windows 7)

3.1 Linux (tested on Ubuntu 16.04 LTS 64-bit)

3.1.1. Configure Eigen:
- Download the .tar.gz archive file for latest stable release of Eigen (Eigen 3.3.4 at the time this README was written) from http://eigen.tuxfamily.org
-  Extract the archive.
-  Open the extracted folder. Follow the instructions in the INSTALL file. Since Eigen is a header template library, there are no libraries to link. 

If this does not work for some reason, you can copy the Eigen folder to one of two places:
  a. /usr/include/eigen3/
  b. The same folder that batotp was extracted to. The folder structure should resemble:
  /PathToParentFolder/time_optimal_trajectory_planning/batotp
  /PathToParentFolder/eigenlib/Eigen

3.1.2. Verify the version of the gcc compiler (In Ubuntu the command for this is "gcc -v". BA was tested using gcc 5.4.1, but versions 4.8.1 and up should work.

3.1.3. Compile BA

a. Using only the gcc compiler
- Open a terminal in the folder that BA was extracted to, which should include the file compile.sh
- Run the command ./compile.sh in the terminal
- The compiled executable is Release/batest

b. Using Qt with the gcc compiler
- Install the latest version of Qt. For Ubuntu, at the time this document was written, the latest version was 5.10.0, available here:
https://download.qt.io/archive/qt/5.10/5.10.0/qt-opensource-linux-x64-5.10.0.run
- During the installation of Qt, at the "Select Components" page, select "Desktop gcc 64-bit" below "Qt 5.10.1" and "Qt Creator 4.5.0" below Tools. Use the default options for everything else. You will need to create a Qt account if you don't already have one.
- Open the file time_optimal_trajectory_planning.pro with Qt Creator. This file is located in the folder where BA was extracted to.
- In the menu at the left, click "Projects" and then uncheck the box for "Shadow build" in the Build Settings (you have to do this for both the Debug and Release build configurations)
- In the menu at the left, click "Edit", then right mouse-click time_optimal_trajectory_planning in the Projects tab and select Build
- The compiled executable is test/batest
- You can run the executable in Qt using the keystroke "CTRL + R", or you can run it separately in another terminal


3.2 Windows (tested on Windows 7 Pro 64-bit)

3.2.1. Configure Eigen:
- Download the .zip archive file for latest stable release of Eigen (Eigen 3.3.4 at the time this README was written) from http://eigen.tuxfamily.org
-  Extract the archive.
-  Open the extracted folder. The necessary source files are all in the Eigen folder. Since Eigen is a header template library, there are no libraries to link.
-  Copy the Eigen folder to the same folder that batotp was extracted to. The folder structure should resemble:
  /PathToParentFolder/time_optimal_trajectory_planning/batotp
  /PathToParentFolder/eigenlib/Eigen

3.2.2. Compile BA
  
a. Using MinGW
- Download and install MinGW:
https://sourceforge.net/projects/mingw/files/latest/download?source=files
- In the MinGW Installation Manager, check the boxes for "mingw32-base" and "mingw32-gcc-g++" and then go to Installation -> Apply Changes
- Open a command window in the folder that BA was extracted to, which should include the file compile.bat
- Run the command compile.bat in the command window. This script assumes that MinGW was installed to C:\MinGW\bin\. If it was installed somewhere else, then modify compile.bat accordingly.
- The compiled executable is test/batest.exe

b. Using Qt and Microsoft Visual Studio
- Install the latest version of Qt. For Windows, , at the time this document was written, the latest version was 5.10.0, available here:
https://download.qt.io/archive/qt/5.10/5.10.0/qt-opensource-windows-x86-5.10.0.exe
- Install Qt
- When you get to the "Select Components" page, under 5.10.0 select "msvc2017 64-bit"
- Follow the rest of the instructions in Section 3.1.3.b above
- The compiled executable is x64/Debug/batest.exe or x64/Release/batest.exe

c. Using Microsoft Visual Studio (version 2013 or later is required)
- BA was tested using Visual Studio 2017 Community
- If necessary, download and install Visual Studio: https://www.visualstudio.com/vs/whatsnew/

For Visual Studio 2017 Community, during installation, select the option "Desktop development with C++"
- At the right, the only required options to compile BA are "VC++ 2017 v141 toolset (x86/x64)" and "Windows 10 SDK (10.x.xxxxx.x) for Desktop C++ [x86/x64]"
- Click "Install" to complete the installation process

- Once Visual Studio has been installed, open the file time_optimal_trajectory_planning.sln
- The subprojects are configured for Visual Studio 2013. If you have a more recent version, you will be asked to retarget the solution, which should normally be accepted.

- Click on the top menu item Build, then Build Solution
- The executable is batest.exe, and it will be located in the Debug or Release folder (these are subfolders of the x64 folder for 64-bit builds)


4. Using BA --------------------------------------------------

Three methods for using BA are described below.

4.1 The bin/batest executable (bin/batest.exe in Windows) and config.dat

The executable can be run from the terminal (command prompt in Windows), or from an .sh script (.bat file in Windows). It can also be called from any software that can issue system commands, such as MATLAB.

Options are specified in the file "../input/config.dat", where the folder location is relative to the batest executable. The input trajectory data file is also specified in config.dat. A detailed description of all configuration options is provided in "input/README_for_config_file.txt".

After running the executable, the output trajectory data file, along with other output files related to algorithm performance, can be found in the "output" folder.

4.2 Command line options

A different configuration file can be specified as a command line option. For example, for the input file "configRR.dat", the command is "batest configRR.dat". In this case, all output files will be generated in the same folder as the batest executable. 

4.3 The batotp library

To integrate BA into other projects, functions can be linked directly from the batotp library. The test subproject which generates the batest executable provides a demonstration of how this is accomplished.

5. Examples --------------------------------------------------

Five examples can be found in the input folder, which correspond to the five robots defined in robot.cpp:

(a) A 2-DOF RR robot with a full dynamic model and dynamic constraints
(b) The KUKA LWR-IV+ 7-DOF serial robot, with joint and Cartesian kinematic constraints (velocity and acceleration)
(c) The UR5 robot, with joint and Cartesian kinematic constraints
(d) A general 7-DOF serial robot, without a kinematic or dynamic model, with constraints on joint velocity and acceleration)
(e) A 3-DOF cable-suspended parallel mechanism (CSPR), with kinematic and dynamic constraints

Any of these examples can be executed by copying the config.dat file from the robot subfolder to the input folder, along with the trajectory data file specified in config.dat.

Detailed descriptions are provided for (d) and (e) below. GNU Octave .m scripts are described below for generating input trajectory data files and for plotting the optimization output. Matlab can also be used to run the scripts. Octave is free, open-source Scientific Programming Language with a syntax is largely compatible with Matlab:

https://www.gnu.org/software/octave/

All .m scripts were tested using Octave 4.2.1. Note that in Windows 7, the first time you try to make a plot with Octave, it might take a while and then switch to a Basic theme. Once it works the first time, it seems to switch to a Basic theme immediately for subsequent plots. You can force Windows to use a basic theme by right-mouse-clicking the Desktop -> Personalize -> Select "Windows 7 Basic"

Older versions of Ubuntu (e.g. 14.04) might install an older version of Octave by default. To install a newer version, the instructions at the following webpage can be adapted:
https://askubuntu.com/questions/645600/how-to-install-octave-4-0-0-in-ubuntu-14-04

5.1 A general 7-DOF serial robot, without a kinematic or dynamic model

- Joint positions can be thought of as unit-less, though for this example, reasonable values for radians are chosen
- Constraints are imposed on joint velocity and joint acceleration
- When joint positions have units, constraints must be specified in consistent units in the config.dat file. In this case, reasonable limits in radians of 5 rad/s for velocity and 10 rad/s^2 for acceleration are chosen for each joint.

5.1.1 Generating trajectory data

Two methods for generating trajectory data are provided:

generateGEN7DOFpathBasic.m - Generates the ASCII input data file "GEN7DOFpathBasic.csv" with random five trajectory points. Joint positions are in the range [0, 5].

generateGEN7DOFpath.m - Generates 20 random traj. points, then uses spline interpolation to create a 400-point trajectory. Generates the input files "GEN7DOFpath0001.csv" and "GEN7DOFpath0001.dat". These files are identical, except one is in ASCII CSV format and one is in binary format. Binary format is preferred, especially for large data sets. Multiple random trajectories can be generated by changing the "nIter" parameter.

The config.dat file specifies "GEN7DOFpathBasic.csv" as the traj. data file, so copy config.dat and GEN7DOFpathBasic.csv to the input folder to optimize this path.

5.1.2 Executing batest / Running the optimization
Ubuntu: Open a terminal in the Release or Debug folder. Run "./batest".
Windows: Open a command prompt in the Release or Debug folder. Run "batest.exe".

5.1.3 Viewing the output trajectory and algorithm performance
- Open Octave or Matlab in the output folder.
- Run plotOutput.m
- Plots are generated of:
  - Fig. 2: The curve in s-sdot after reverse integration and then after forward integration
  - Fig. 3(a): Output joint velocity vs. time
  - Fig. 3(b): Output joint accel.   vs. time

5.2 A 3-DOF cable-suspended parallel mechanism (CSPR), with kinematic and dynamic constraints
- Position data are specified as (x,y,z) coordinates in metres
- Constraints are imposed on joint velocity (cable velocity), joint acceleration (cable acceleration), Cartesian speed, and cable tension

5.2.1 Generating the trajectory data
Open and run the file "input/CSPR3DOF/generatePathPointsCSPR.m" in Octave or Matlab. The binary trajectory data file "CSPR3DOFspline.dat" will be generated. Copy "config.dat" and "CSPR3DOFspline.dat" to the input folder.

5.2.2 Executing batest / Running the optimization
Ubuntu: Open a terminal in the Release or Debug folder. Run "./batest".
Windows: Open a command prompt in the Release or Debug folder. Run "batest.exe"

5.2.3 Viewing the output trajectory and algorithm performance
- Open Octave or Matlab in the output folder.
- Run plotOutput.m
- Plots are generated of:
  - Fig. 1: A 3D plot of the trajectory in space
  - Fig. 2: The curve in s-sdot after reverse integration and then after forward integration
  - Fig. 3(a): Output cable velocity vs. time
  - Fig. 3(b): Output cable accel.   vs. time
  - Fig. 4(a): Output Cartesian speed vs. time
  - Fig. 4(b): Output Cartesian accel. vs. time
  - Fig. 5: Output Cable tension vs. time

6. Defining new robots for BA --------------------------------------------------

The easiest way to use a new robot in BA is to add it to robots.cpp. There, kinematics and dynamics functions for other robots are provided, which can be used as starting points.


