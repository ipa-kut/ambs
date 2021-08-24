# AMBS

## AUTOMATED MODULAR BENCHMARKING SYSTEM

Successor to the [ARAIG Test Stack](https://github.com/ipa320/araig_test_stack). The ARAIG project aims at developing a modular testing software kit to run tests and benchmarks primarily on mobile robots developed by various partner institutes. These tests are aimed at benchmarking middle level performance attributes such as speed profile, braking distance, stability over terrain etc...; as well as testing high level functional features such as object avoidance, navigation efficiency, object identification etc... This software stack is to be complimented by hardware elements such as the robot itself & a testing floor; as well as external sensors such as cameras, indoor positioning systems, IMUs etc...

The fundamental design philosophy for this software toolkit is modularity. This enables us to quickly and easily put together a brand new test by utilizing existing code as much as possible, and reducing the need for extra coding. To achieve this, we leverage the inherently modular nature of ROS - Nodes & Topics. The AMBS/ARAIG stack consists of several "components", which are ROS nodes, that can be put together into a cohesive system through a launch file, which performs the test as we need. All communication is over ROS topics, using mostly an "event" based system for control & coordination, and a stream of "data" for computation.

Keeping this in mind, although the main idea is to use this software kit to test mobile robots, in principle, it can used to create mid/high level tests and benchmarks for any ROS based system.

There are 5 main types of components which all work together to form a test system:
* *Calculators* - Atomic units that specialise in one single computation. Adhere to a minimum standard interface. Combine these to achieve more complex computation. An initial library is offered, but more can be easily developed with standard interfaces as required, based on the [abstract class](https://github.com/ipa-kut/ambs/blob/master/ambs_core/include/ambs_core/ambs_base_calculator/ambs_base_calculator.hpp)
* *Interpreters* - Adaptors that bridge the standard interfaces used within AMBS/ARAIG to external ROS interfaces such as robots drivers & sensors. Highly unique per external device, and must be completely handwritten. Complexity depends on device, but can be reused.
* *Runners* - Simple scripts that orchestrate a test process such as trigger timings & final test result decision. Highly unique per test, and must be completely handwritten, but are also very simple to write based in the [abstract class](https://github.com/ipa-kut/ambs/blob/master/ambs_core/include/ambs_core/ambs_base_runner/ambs_base_runner.h)
* *Loggers* (not yet implemented) - These nodes are responsible for saving data into the filesystem for analysis and post-processing.
* GUI (under development) - A simple GUI to interact with the runners.

## Automation in testing

The "Automated" in AMBS stems from the idea of using the [ROS Models tooling system](https://github.com/ipa320/ros-model). This toolchain can provide several advantages for *design-time* automation: (*TODO: @ipa-rwu*)
* Extract target system interfaces (Robots + sensors)
* Compose test system launch file
* Generate test package
* Generate new calculator templates

Apart from this, the *Runner* scripts are meant to provide a high degree of *run-time* automation, often requiring the end user to only start the test. The test is then autonomously executed and final results are computed and logged.

## AMBS vs ARAIG

The ARAIG stack was based mostly on Python. This made rapid prototyping easier for a good proof of concept. However, we believe that the inherent nature of the rospy API introduces some message queuing limitations which we hope to overcome by using the more robust roscpp API. Benchmarking results of the test stack itself will eventually provide more conclusive evidence. 

The ARAIG stack is developed for actual use with partner clients for testing, and has been refined for practical use. The AMBS stack is currently still under development and aims to provide the minimum feature set required for benchmarking itself.

More results to follow (*TODO: @ipa-kut*)

## Code Documentation

Please see the Doxygen [docs](https://ipa-kut.github.io/ambs/html/index.html) for detailed descriptions of components and an architecture overview.

## Package descriptions

### ambs

The meta package to encapsulate together this complete software stack

### ambs_msgs

Contains ROS msgs. Serve as standard interface types within the AMBS system.

### ambs_core

Core package contains base classes for all components.

### ambs_components

Contains header files defining classes for all components. This serves as the main logic library where all additional components developed must also be stored.

### ambs_pluginlib

Contains the nodelet code of components, which implements classes defined in `ambs_components`. Instances of these nodelets can be launched together  to compose a test system. Contains launch files for interactive testing of components, and also template examples of basic test systems.

### ambs_tests

Contains automated unit tests for components. These tests also give a good idea of how components should be used.

### ambs_sim_tests

Contains TurtleBot3 simulation testing environment for testing a complete AMBS test.

# Developer Info

## Linting

First add roslint to CMakeLists.txt & package.xml of all packages that need to be linted (see `ambs_core` for example)
Then, to run lint, for example for the `ambs_core` package:   

`catkin_make roslint_ambs_core`

## Testing

To build tests: `catkin_make tests`   
To build & run tests: `catkin_make run_tests`   
To run individual tests verbosely: `rostest ambs_tests test_XXX.test --text`   

