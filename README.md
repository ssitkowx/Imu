# Template

# I. Description:
Template for projects and packages.

# II. Assumptions:
The code stored here is a generic code, which means:
- The code should be dependent only on C/C++ language libraries,
- The code should work in various environments (Linux, Windows, Embedded) and be independent of them,
- The code can use only generic external libraries,
- The conan scripts were adopted to Linux.

# III. Structure:
The solution project has been divided into three parts:
- Project,
- Project library,
- Tests which uses the project, GTest and/or GMock libraries.

# IV. Configuration:
- Python 2.11.6 with required packages,
- Conan 2.2.2
- Git 2.40.1
- CMake 3.16.3
- Visual Studio Code 1.88.0
- Conanfile.py should be updated according to the example below:
  - name        = "Template"                            -> Display
  - Packages    = ["packageName/version", next package] -> ["Logger/1.0", "Utils/1.0"]
  - description = "Template for projects and packages"  -> "General class for display"

# V. Builidng:
- Go to 'Conan' folder and open git bash console,
- Type 'conan install . --build=gtest/cci.20210126',
- Type 'conan source .',
- Type 'conan build .'

# VI. Tips:
- When making new changes to the package, delete the copy of the local repository (../Repos/"PackageName") and replace it with a new one
When using with VCode:
- Complete all steps form point V,
- Open VCode and when he ask select the conan-release configuration.
