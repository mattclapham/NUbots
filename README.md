# NUClear Roles
The NUClear roles system is a build and messaging system for the NUClear framework.
It uses on CMake and Python to manage the construction of various executables made up of a selection of modules.
These executables are called roles.

CMake is used as the main system for generating the libraries and executables that are used for the final binary.
Note that it utilises globbing to find the sources that are used for modules.
So if you add or remove a file, make sure you rerun cmake so that it locates the new files.

## Setup
NUClear Roles is designed to exist as a part of another repository as either a git subtree or git submodule.
In general subtrees are preferred to submodules as they allow you to make your own local changes to NUClear Roles and still be able to merge in upstream changes.
To setup NUClearRoles as a subtree follow the following steps.

- First create your repository where the NUClear Roles based system will live.
- Once you have a repository to attach to run the following command from the root directory of the repository
```bash
git subtree add --prefix nuclear https://github.com/Fastcode/NUClearRoles.git master --squash
```
- This will pull in NUClear Roles into the nuclear subdirectory ready for use by your system.

Once you have added NUClearRoles to your codebase you must then configure a CMakeLists.txt file to use it.
The structure of this CMakeLists.txt file should be as follows

```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(<Project Name Here>)

# Set variables for NUClear Roles here, e.g. to set the banner file to a custom banner
SET(NUCLEAR_ROLE_BANNER_FILE "${PROJECT_SOURCE_DIR}/banner.png" CACHE PATH "The path the banner to print at the start of each role execution" FORCE)

# Finally Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)
```

CMake code that influences the variables used by NUClear Roles should come before the `ADD_SUBDIRECTORY` line.
CMake code that depends on variables created by NUClear Roles should come after that line.

### Dependencies
NUClear roles has several dependences that must be met before you are able to build the system.
These dependencies are:
- NUClear
- Python3 with the following packages
  - argparse
  - Pillow
- And optionally for Python support pybind11

### Banner
NUClear roles generates an ansi coded banner at the top of ever role it runs.
This banner file is created from an image file that is provided using the CMake cache variable `NUCLEAR_ROLE_BANNER_FILE`

As the banner file is converted into ansi coloured unicode text, there are limitations on how the final result can look.
To ensure that your banner looks good when rendered you should consider the following advice.
- Ensure that your image is 160px wide or less.

    Many terminals when created are 80 columns wide.
    The resolution of the created unicode text is half the resolution of the image.
    This means a 160px wide image will make 80 columns of text.

- Try to make your logo in an editor using a 0.5 pixel aspect ratio.

    The resolution of the image will be divided by four for the vertical axis.
    This is done as text blocks are (almost) twice as high as they are wide.

- Try to make smooth gradients.

    When selecting colours for the text, each character can have 2 colours which are aranged into any combination of the 4 quadrants.
    The unicode characters `█ ▄ ▐ ▞ ▟ ▚ ▌ ▙ ▀ ▜ ▛` are used to colour images so if the image has complex gradients the combination of these will look less clear.

If you do not set your own banner file, ![the default banner](roles/banner.png) will be used

## Directories
There are four main directories that are used within the NUClear Roles system.
These are the module, extension, message, utility and roles directories.
The code that makes up your system will be primiarly stored within these directories.

### Module
The module directory is where all NUClear Reactors will be stored.
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_MODULE_DIR`.
If this variable is not set it defaults to `module`.

Within a the modules folder, a strict directory structure must be maintained so that the NUClear Roles system can find and build your modules.
Take an example module `Camera` which exists in `namespace input`.
This module must be located at `${NUCLEAR_MODULE_DIR}/input/Camera`.
Within this module folder there are three directories that may hold code for the system.
- `${NUCLEAR_MODULE_DIR}/input/Camera/src` holds all of the source code for the module
  - This directory must contain a header file with the same name as the module followed by hpp, hh or h. E.g. Camera.h and this header must declare the NUClear::Reactor with the same name.
- `${NUCLEAR_MODULE_DIR}/input/Camera/data` holds any non source code files that are required. These will be copied to the build directory when building the code.
- `${NUCLEAR_MODULE_DIR}/input/Camera/test` holds any unit test source code.

### Message
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_MESSAGE_DIR`.
If this variable is not set it defaults to `shared/message`.
It is highly recommended that the message, utility and extension folders share a common parent folder.

`TODO`


#### Neutron Messaging System

`TODO`


### Utility
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_UTILITY_DIR`.
If this variable is not set it defaults to `shared/utility`.
It is highly recommended that the message, utility and extension folders share a common parent folder.

`TODO`


### Extension
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_EXTENSION_DIR`.
If this variable is not set it defaults to `shared/extension`.
It is highly recommended that the message, utility and extension folders share a common parent folder.

`TODO`


### Roles
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_ROLES_DIR`.
If this variable is not set it defaults to `roles`.

## B SCRIPT
`TODO` module generate
`TODO` custom tools


## NUClear Modules
`TODO`

Variables
INCLUDES
LIBRARIES
SOURCES
DATA_FILES

## Roles

`TODO`
