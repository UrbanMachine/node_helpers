# Using `create-ros-app`

This repository was initialized by the [create-ros-app](https://github.com/UrbanMachine/create-ros-app) template.
This template is a everything-but-the-robot-code starter for ROS projects. It includes a Dockerfile for building ROS packages, a GitHub Actions workflow for linting and autoformatting, and a few other goodies.

This documentation walks through the features of the template, and how to use them.

**Make sure to follow the [Post-Set-Up Guide](https://github.com/UrbanMachine/create-ros-app/blob/main/README.md#post-set-up-guide) after setting up the template before diving into the features below!**

## Quick Start Guide

Here's a quick guide on the features of this template

### Scripts
- Linting and autoformatting for python (ruff), C++ (clangformat), bash (shellcheck), and javascript (eslint)
  
  **Relevant Scripts:**
  ```shell
  poetry run lint --help
  ```
  You may need to run `poetry lock` and `poetry install` before running the above command.
- Easily build ROS apps in an out-of-the box containerized environment, with helper scripts under the `docker/` directory.
  
  **Relevant Scripts:**
  ```shell
  # Run a profile specified under `launch-profiles/`
  docker/launch <profile>
  
  # Run and enter a new ROS container without executing anything
  docker/run
  
  # Enter a currently running ROS container to poke around
  docker/exec
  ```
  
  More usage examples for the above scripts are documented at the top of the script files.

### Things You Likely Want To Do

- **Add new dependencies:**
  - **Python dependencies:** can be added under `pkgs/<package_name>/pyproject.toml`. Run `poetry lock` after. 
  - **ROS dependencies:** can be added under `pkgs/<package_name>/package.xml`.
  - **System dependencies:** can be added under `docker/Dockerfile`, under the `install-packages` section.
  
  After any changes, the container will automatically re-build on the next launch.
- **Save persistent data:** The `/robot/persistent` directory is mounted to `.docker_volumes/ros-nodes/` on your local machine. Save data there to persist across container runs, and to view it in your file manager.
- **Look at logs:** Logs are available at `http://localhost/` after running `docker/launch <profile>`.
- **Create a new Launch Profile:**
  - Add a directory under `docker/launch-profiles/`
  - Add a `launching.py` file that launches your ROS nodes inside your new profile directory
  - Fill out the `launching.py` file with the nodes you want to launch.
  
## Project Structure

### `pkgs/`

The packages directory contains all the packages that are used in the project. Each package is added in the `Dockerfile`, and any new packages should be added there as well.

#### Package structure
Each package is made up of:
- A `resource` directory, which is a colcon requirement
- A `package.xml` file, which is a colcon requirement
- A `pyproject.toml`, because this project uses [colcon-poetry-ros](https://github.com/UrbanMachine/colcon-poetry-ros) to install project dependencies. Most ROS python packages use `setup.py`, but by using this plugin, we can use a modern python tool called [Poetry](https://python-poetry.org/) to manage dependencies.
- A directory for code
- A directory for tests

#### Test directories

As (arbitrary) best practice, the example node uses a test directory that follows the following structure

```shell

package_name/
├── package_name/
│   ├── __init__.py
│   ├── node.py
├── package_name_test/
│   ├── unit/ 
│   │   ├── test_node.py
│   ├── integration/
│   │   ├── test_node.py

```

Essentially, tests exist in a parallel directory to the package, and are split into `unit` and `integration` tests. The directories within `unit` and `integration` mirror the structure of the package itself, except that module names are prefixed with `test_`.

### `.github/`

This project uses poetry for linting, and has some code for running linting and autoformatting under `.github/lint`.
Run `poetry install` then enter the shell via `poetry shell` to get access to the linting tool.

This project is based on a template, upstream changes can be found on the [template repository](https://github.com/UrbanMachine/create-ros-app).
A tool called `cruft` will alert you when there are upstream changes, and help you merge those in.

### `docker/`

This directory contains scripts for building and running the Docker container. Look at the [Fancy Features](#fancy-features) section for more information on how to use these scripts.

As for the structure:

```shell
docker/
├── grafana/   # Stores Grafana provisioning configuration
├── promtail/  # Stores Promtail provisioning configuration
├── utils/     # Contains utility scripts for building or running inside the Docker container
│   ├── environment/  # Scripts for use during Dockerfile build
│   ├── runtime/      # Helper for use when using the docker container
```

#### `docker/Dockerfile`

Feel free to edit as you like, but it's recommended to stick to editing within the sections
blocked off by `#######`. 

The Dockerfile will need to be edited in two places for each new ROS package you add:


1. Copying in the `package.xml`
2. Copying in the `pyproject.toml` and `poetry.lock`

There's also a location for adding new `apt` dependencies. It's recommended that ROS package
dependencies are added through `package.xml` if they are available.

### `.env`

This file holds environment variables that are used by the docker build and launch scripts.

Most variables can safely be edited.

### `.docker_volumes/`

This directory is automatically created the first time you run `docker/launch`. It holds persistent state across runs for containers. 

For example, if you enter your container via `docker/run` and save a file under `/robot/persistent/`, that file will be available the next time you run `docker/run`. It will also (by default) exist in your local machine under `.docker_volumes/ros-nodes/`.
The `/robot/persistent` directory is intended for you, the developer, to use. So have at it!