# node_helpers
An opinionated ROS2 framework that minimizes boilerplate while maximizing reliability. Features intuitive APIs for parameter management, action handling, and error-resilient RPC. Designed by Urban Machine for safe and scalable robotics.

---

[![Test Status](https://github.com/urbanmachine/node_helpers/workflows/Test/badge.svg?branch=main)](https://github.com/urbanmachine/node_helpers/actions?query=workflow%3ATest)
[![Lint Status](https://github.com/urbanmachine/node_helpers/workflows/Lint/badge.svg?branch=main)](https://github.com/urbanmachine/node_helpers/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/urbanmachine/node_helpers/branch/main/graph/badge.svg)](https://codecov.io/gh/urbanmachine/node_helpers)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros-%230A0FF9.svg?logo=ros&logoColor=white)

---

## Running This Project

For in-depth documentation on the repository features, read the [About Template](docs/about_template.md) documentation.

### Dependencies

This project depends on [Docker](https://docs.docker.com/get-docker/). The linting tooling requires [Poetry](https://python-poetry.org/docs/) to run.

### Running the Project

This project is intended to be used as a library, however there is a showcase project 
that demonstrates some of the library's features.

To run the project, use the following command:

```shell
docker/launch node_helpers_showcase
```

Take a look at the nodes under `pkgs/node_helpers/nodes`, and the launch files under 
`pkgs/node_helpers/launch` to get an idea of some of the libraries features.

Then, open http://localhost/ on your browser to view the project logs.



---
This repository was initialized by the [create-ros-app](https://github.com/UrbanMachine/create-ros-app) template. Contributions are welcome!