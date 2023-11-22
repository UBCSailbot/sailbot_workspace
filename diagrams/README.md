# PlantUML

## Overview

The simplest way to describe PlantUML is "diagrams written with code". The main benefit of using PlantUML instead of
making diagrams in GUI-based software like draw.io or Visio is the ability to version control diagrams. Since they are
rendered based on plain text descriptions, Git can easily track edit histories. Another benefit is the easy propagation
of an organization specific diagram style. By including [template.puml](template.puml) in other puml files, we can have
a consistent style across sailbot_workspace. Simply add `!include %getenv("PLANTUML_TEMPLATE_PATH")` to your file.

## How To

### Render In VSCode (Simplest Way to View)

When you have a `.puml` file open in VSCode, there will be an icon in the top right of the tab with a magnifying glass.
Simply click it and the diagram will render to the side. Usually, it automatically updates whenever you make a change to
the `.puml` file. Occasionally, you'll need to run `PlantUML: Preview Current Diagram` in the VSCode command palette
(`ctrl+shift+p`) to manually update the rendered diagram.

### Export As Image

Exporting as an image is required for viewing PlantUML diagrams on services like Google Drive and Confluence.

If simply copying an image suffices, then [render in VSCode](#render-in-vscode-simplest-way-to-view), hover over the
rendered diagram and click the copy icon to the left of the help button. You can also export it as a PNG
by opening the VSCode command palette (`ctrl+shift+p`) and running one of the
`PlantUML: Export <...>` commands. The exported diagram(s) can be found under `diagrams/out/`.

### Create a Diagram

PlantUML supports various diagram types. This section goes over a few types in use by the Sailbot Software Team and
describes their use cases. For a full list, see the [PlantUML website](https://plantuml.com/). Examples of the diagrams
described below can be found under [src/](src/).

#### [Sequence Diagram](https://plantuml.com/sequence-diagram)

Use when describing the control sequence/flow of a module or algorithm. Its usecase is similar to flow diagrams
[PlantUML calls these activity diagrams](https://plantuml.com/activity-diagram-beta), but has some key advantages:

- Clearly organize different modules and reduces. Each module and resource can have their own category in a diagram, and
can be grouped with related modules and resources.
- Explicitly show resource usage. When a flow accesses a resource persistent across sequences, accesses can be shown
and described as part of the flow. For example, showing reads, write/appends, allocations, and clears.
- Represent simultaneous control flows. When exploiting parallelism and asynchronous operations, multiple modules can
be shown as active and described in a diagram at once.

The primary limitation of sequences diagrams is non-trivial branching (ex. if/else flows). Sequence diagrams are
clearest when the control flow has one possible path. To handle non-trivial branching, consider:

- Splitting branches into their own sequences/subsequences. Dedicate a sequence/subsequence to when a branch is true and
another where the branch is false.
- Using a [flow/activity diagram](https://plantuml.com/activity-diagram-beta).

#### [State Diagram](https://plantuml.com/state-diagram)

Primarily used for state machines, but can also be used to show how nodes in a system are connected (ex. ROS topics,
publishers, subscribers).

## Resources

- The [PlantUML website](https://plantuml.com/) is an excellent resource for how to create diagrams. Each tutorial page
is relatively short and goes over every diagram specific feature and some basic styling.
- For more advanced styling, comprehensive documentation can be found
[here](https://plantuml-documentation.readthedocs.io/en/latest/formatting/all-skin-params.html).
- The [src/](src/) folder contains some basic diagram examples that utilize the template.
