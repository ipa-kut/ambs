# AMBS
Automated Modular Benchamrking System - Successor to the ARAIG Test Stack

## Package descriptions

### ambs_msgs

Contains msgs, srvs & actions.
### ambs_core

Core package contains base classes for use by others

### ambs_components

Contains header files defining classes for all components, uses `ambs_core`

### ambs_plugliblin

Contains the actual nodelet implementations, which implements classes defined in `ambs_components`   
Contains also launch files to quickly test nodelets.   

## Code Documentation

See the Doxygen [doc](doc/html/index.html) for detailed descriptions and hierarchy overview.
## Linting

First add roslint to CMakeLists.txt & package.xml of all packages that need to be linted (see `ambs_core` for example)
Then, to run lint, for example for the `ambs_core` package:

`catkin_make roslint_ambs_core`
