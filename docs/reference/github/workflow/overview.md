# Development Workflow Overview

``` mermaid
graph LR
    B[Problem Conception] --> C{Small Fix?};
    C --> |Yes| E[Development];
    C --> |No| D[Issue Creation];
    D --> E;
    E --> F[Pull Request];
    F --> G{Approved?};
    G --> |No| E;
    G --> |Yes| H[Merge PR into Main];
```

A good development workflow is essential to maintain a robust codebase and stay organized. The above
diagram is a high level overview of how our development process works, and parts of this process
are explained in subsequent sections.

## Version control: Git

We use git to help us keep track of the version history of our codebase. Git is a free and open source
distributed version control system, and it is commonly used by many developers to keep track of changes
to their code over time. As a member of the software team on UBC Sailbot, it is absolutely necessary that
you know git. If you are unfamiliar with git, here are a few resources to help you get started:

| Resource                                               | Description                                            |
| :----------------------------------------------------- | :----------------------------------------------------- |
| [Beginners Tutorial](https://youtu.be/HVsySz-h9r4){target=_blank}     | A 30 minute video on git for beginners. Good if you want to learn git quickly and nail all the fundamentals.    |
| [Pro Git book](https://git-scm.com/book/en/v2){target=_blank}         | A textbook on using git. Good if you are a completionist and want to deep dive into how git works (and if you have some time on your hands).    |
| [Common Git Commands](https://patrick-5546.github.io/notes/reference/git/git_commands/){target=_blank} | A condensed summary of some common git commands. Good to refer to once you are familiar with the fundamentals of git. |

## Remote server: GitHub

We use GitHub as our remote server where we store our codebase. In addition to using it for storage, we also
leverage many of GitHub's features to make for a smoother development process. Some examples of features
that we use are:

- [Issues](./issues.md#creating-issues)
- [Projects](./issues.md#adding-issues-to-a-project)
- [Milestones](./issues.md#adding-issues-to-a-milestone)
- GitHub Organizations
- Repository Permissions and Branch Protection Rules
- And more!
