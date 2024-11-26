# node_helpers.interaction

This module provides a framework for creating interactive menu systems within ROS 2 applications. It is designed to simplify the process of generating dynamic menus and prompts that allow users to make selections, control actions, and provide input during runtime. 



## Module Components
###Base Classes

### BaseMenu

A foundational class for creating interactive menus. It provides mechanisms to display options, capture user input, and link selections to specific callbacks. It also supports running cancelable actions, making it suitable for scenarios where operations need to be interrupted based on user decisions.

### BasePrompter
An abstract base class that handles the logic of displaying prompts and waiting for user input. It defines methods for connecting to a ROS system, interpreting user messages, and managing subscriptions or services for input.

### Implementations

#### DashboardMenu
Extends the functionality of the base menu by integrating with a web-based dashboard. It uses ROS topics and services to display menus and capture responses, making it ideal for remote user interfaces.

#### DashboardPrompter
Implements the prompter functionality for the web UI. It allows users to make selections through a dashboard and manages the lifecycle of prompts and responses.