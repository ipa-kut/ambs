# AMBS
Automated Modular Benchamrking System - Successor to the ARAIG Test Stack

## Package descriptions

### ambs_msgs

Contains msgs, srvs & actions.
### ambs_core

Core package contains base classes for use by others

## Linting

First add roslint to CMakeLists.txt & package.xml of all packages that need to be linted (see `ambs_core` for example)
Then, to run lint, for example for the `ambs_core` package:

`catkin_make roslint_ambs_core`
