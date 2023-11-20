# PlantUML

## Overview

The simplest way to describe PlantUML is "diagrams written with code". The main benefit of using PlantUML instead of
making diagrams in GUI-based software like draw.io or Visio is the ability to version control diagrams. Since they are
rendered based on plain text descriptions, Git can easily track edit histories. Another benefit is the easy propagation
of an organization specific diagram style. By including [template.puml](template.puml) in other puml files, we can have
a consistent style across sailbot_workspace. Simply add `!include %getenv("PLANTUML_TEMPLATE_PATH")` to your file.

## How to Run

When you have a `.puml` file open in VSCode, there will be an icon in the top right of the tab with a magnifying glass.
Simply click it and the diagram will render to the side.

## Resources

- The [PlantUML website](https://plantuml.com/) is an excellent resource for how to create diagrams. Each tutorial page
is relatively short and goes over every diagram specific feature and some basic styling.
- For more advanced styling, comprehensive documentation can be found
[here](https://plantuml-documentation.readthedocs.io/en/latest/formatting/all-skin-params.html).
- The [examples/](examples/) folder contains some basic diagram examples that utilize the template.
