# Generic Navigation (ROS 2)

## 1. Installation

## 2. 

## 3. CMake

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


### ament_cmake 
* [ament_cmake](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html): `ament_cmake` is the build system for CMake based packages in ROS 2. 
* The project setup is done by `ament_package()` and this call must occur exactly once per package. `ament_package() installs the package.xml, registers the package with the ament index, and installs config (and possibly target) files for CMake so that it can be found by other packages using find_package.  
* `ament_export_include_directories(include)`  marks the *directory* of the exported include directories (this is achieved by `INCLUDES DESTINATION` in the target `install` call). `ament_export_libraries(my_library)` marks the location of the installed library (this is done by the `HAS_LIBRARY_TARGET` argument in the call to `ament_export_targets`).
* `ament_export_dependencies` exports dependencies to *downstream packages*. This is necessary so that the user of the library does not have to call find_package for those dependencies, too.

### ament_cmake_python
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

## C++

### KEYWORDS
* [enum](https://www.programiz.com/cpp-programming/enumeration): An enumeration is a *user-defined* data type that consists of **integral constants**/In C++ programming, enum or enumeration is a data type consisting of *named values* like elements, members, etc., that represent **integral constants**. -> used when you expect the variable to select one value from the possible set of values. It increases the abstraction and enables you to focus more on values rather than worrying about how to store them. It is also helpful for code documentation and readability purposes.

* [namespace](https://www.geeksforgeeks.org/namespace-in-c/): separate scopes for names.

* [typedef](https://www.cprogramming.com/tutorial/typedef.html): allows the programmer to create *new names for types* (such as int or more) -> provide more *clarity* to your code and to make it easier to *make changes to the underlying data types* that you use. 

## Gazebo
* [To import a model without export more GAZEBO_PATH](https://automaticaddison.com/how-to-load-a-robot-model-sdf-format-into-gazebo-ros-2/): Using `spawn_entity.py` of `gazebo_ros`
