# Generic Navigation (ROS 2)

## 1. Installation

    ```sh
    # Inside your /src folder of ROS 2 workspace
    git clone https://github.com/ChuongNg1996/generic-navigation-ros2
    cd ..
    colcon build
    ```

## 2. Construction

### 2.1 Pipeline 
#### 2.1.1 VERSION 1: generic-navigation-1-ros2
* User -> **Global PLanner** (`generic_nav_global_planner_1_ros2`).
* **Global PLanner** (`generic_nav_global_planner_1_ros2`) -> **Local PLanner/Path Tracking Controller** (`generic_nav_controller_1_ros2`).
* **Static Map** (`generic_nav_map_1_ros2`) -> **Global PLanner**
* **Localization** (`generic_nav_localization_1_ros2`) -> **Global PLanner** &&  **Local PLanner/Path Tracking Controller**
* **Local PLanner/Path Tracking Controller** -> **Robot Base**.

### 2.2 Global Planner
* Can either take a predfined trajectory (`custom_trajectory.cpp`) or generate a trajectory based on user's goal and given map. 

### 2.3 Local PLanner/Path Tracking Controller
#### 2.3.1 Carrot Planner
* Algorithms
    ```sh
    + Constantly check and correct heading (to the sub goal) first 
        -> Normalize heading from 0 to 2*PI (heading direction is counterclockwise)
        -> If the heading is not reached, check whether required heading is larger or smaller than current heading.
            -> If required heading is LARGER, then ROTATING RIGHT is the sum of (1)[magnitude of current heading] and 
            (2)[offset from 2*PI to required heading]; ROTATING LEFT is the offset from required heading to current
            heading.
            -> If required heading is SMALLER, vice versa.
                -> If ROTATING RIGHT is shorter, then rotate right and vice versa. 
    + Once heading is near enough, move forward till near enough.
        -> Since heading is constantly checked, moving forward will be stopped to prioritize for heading correction
        if the heading deviation exceed the tolerance.
    ```
 
## 3. Examples

### 3.1 Running a Custom Trajectory
    ```sh
    # Run Simulation Environment
    ros2 launch generic_nav_examples_1_ros2 navigation_simulation_launch.py
    # Run Localization
    ros2 run generic_nav_localization_1_ros2 ekf_1_ros2 
    # Run Local Planner
    ros2 run generic_nav_controller_1_ros2 carrot_ctrl_1_ros2
    # Run Custom Trajectory (in Global PLanner)
    ros2 run generic_nav_global_planner_1_ros2 custom_trajectory 
    # Observe the robot runs in trajectory
    ```

## 4. Additions

### 4.1 CMake (Build System)

* [find_package](https://cmake.org/cmake/help/latest/command/find_package.html): Find a package. **Module mode**: CMake searches for a file called `Find<PackageName>.cmake`, looking first in the locations listed in the `CMAKE_MODULE_PATH`, then among the *Find Modules* provided by the CMake installation.  
* [set](https://cmake.org/cmake/help/latest/command/set.html): Set a normal, cache, or environment *variable* to a given value. *Multiple arguments* will be joined as a semicolon-separated list to form the actual variable value
    ```sh
    set(<variable> <value>... [PARENT_SCOPE]) # Set Normal Variable
    set(<variable> <value>... CACHE <type> <docstring> [FORCE]) # Set Cache Entry
    set(ENV{<variable>} [<value>]) # Set Environment Variable
    ```
* [include_directories](https://cmake.org/cmake/help/latest/command/include_directories.html): Add **include** directories to the build. Add the given directories to those the compiler uses to *search for include files*. 
    ```sh
    include_directories([AFTER|BEFORE] [SYSTEM] dir1 [dir2 ...])
    ```
* [add_subdirectory](https://cmake.org/cmake/help/latest/command/add_subdirectory.html): Add a subdirectory to the build. The source_dir specifies the directory in which the source `CMakeLists.txt` and code files are located. 
    ```sh
    add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
    ```
* [add_library](https://cmake.org/cmake/help/latest/command/add_library.html): Add a library to the project using the specified *source files*. `STATIC`, `SHARED`, or `MODULE` may be given to specify the type of library to be created. `STATIC` libraries are archives of object files for use when linking other targets. `SHARED` libraries are linked dynamically and loaded at runtime. `MODULE` libraries are plugins that are not linked into other targets but may be loaded dynamically at runtime using dlopen-like functionality.
    ```sh
    add_library(<name> [STATIC | SHARED | MODULE]
            [EXCLUDE_FROM_ALL]
            [<source>...])
    ```


#### 4.1.1 ament_cmake 
* [ament_cmake](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html): `ament_cmake` is the build system for CMake based packages in ROS 2. 
* The project setup is done by `ament_package()` and this call must occur exactly once per package. `ament_package() installs the package.xml, registers the package with the ament index, and installs config (and possibly target) files for CMake so that it can be found by other packages using find_package.  
* `ament_export_include_directories(include)`  marks the *directory* of the exported include directories (this is achieved by `INCLUDES DESTINATION` in the target `install` call). `ament_export_libraries(my_library)` marks the location of the installed library (this is done by the `HAS_LIBRARY_TARGET` argument in the call to `ament_export_targets`).
* `ament_export_dependencies` exports dependencies to *downstream packages*. This is necessary so that the user of the library does not have to call find_package for those dependencies, too.

#### 4.1.2 ament_cmake_python
* [ament_cmake_python](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Python-Documentation.html): `ament_cmake_python` is a package that provides CMake functions for packages of the `ament_cmake` build type that contain *Python code*. 
* The outline of a package looks like:
    ```sh
        .
    └── my_project
        ├── CMakeLists.txt
        ├── package.xml
        └── my_project
            ├── __init__.py
            └── my_script.py
    ```
* The `CMakeLists.txt` should contain:
    ```sh
    find_package(ament_cmake_python REQUIRED)
    # ...
    ament_python_install_package(${PROJECT_NAME})
    ```
### 4.2 Programming
#### 4.2.1 C++

##### KEYWORDS
* [enum](https://www.programiz.com/cpp-programming/enumeration): An enumeration is a *user-defined* data type that consists of **integral constants**/In C++ programming, enum or enumeration is a data type consisting of *named values* like elements, members, etc., that represent **integral constants**. -> used when you expect the variable to select one value from the possible set of values. It increases the abstraction and enables you to focus more on values rather than worrying about how to store them. It is also helpful for code documentation and readability purposes.

* [namespace](https://www.geeksforgeeks.org/namespace-in-c/): separate scopes for names.

* [typedef](https://www.cprogramming.com/tutorial/typedef.html): allows the programmer to create *new names for types* (such as int or more) -> provide more *clarity* to your code and to make it easier to *make changes to the underlying data types* that you use. 

### 4.3 Gazebo (Simulation)
* [To import a model without export more GAZEBO_PATH](https://automaticaddison.com/how-to-load-a-robot-model-sdf-format-into-gazebo-ros-2/): Using `spawn_entity.py` of `gazebo_ros`
