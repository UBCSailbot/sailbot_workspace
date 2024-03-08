# Developing on Branches

We use branching to work on issues without modifying the main line. This ensures that the main line only
contains functional code and handles merge conflicts that arise
when multiple people are developing at the same time. For a quick rundown on branching in git,
consult the official [git documentation](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell){target=_blank}.

## Creating a branch

When starting a new issue, you will want to create a new branch for it:

!!! caution

    When creating branches locally, it uses your **local copy** to create the new branch. Remember to do a `git pull` 
    if you intend on using the latest changes from the remote branch you are creating from.

```bash title="Creating a new branch from main"
# Switch to main
git switch main

# Update your local copy
git pull

# Clone a new branch from main
git switch -c <branch_name>
```

**IMPORTANT:** When creating a new branch for an issue, you must create the branch from `main`.

## Branch naming convention

When working on a new issue, you will want to create a branch to work on it. We have the following branch
naming convention:

```
user/<github_username>/<issue_number>-<issue_description>
```

!!! example

    If Jill (GitHub Username: jill99) is going to take on an issue titled "Fix bug on pathfinding software"
    and the issue number is 39, then the branch named can be named something like `user/jill99/39-fix-pathfinding-bug`.

If the branch that you are creating is not tied to an issue, then you **do not** need to put an issue number.
A descriptive title will suffice.

## Tracking and committing changes

All files where new changes have been made must first be "staged" in order to make commits:

```
git add <FILES>
```

Files that are staged will be part of your next commit. Once you are confident in your changes and you are ready
to finalize them, then you should commit your changes:

```
git commit -m "<commit_message>"
```

Be sure to add a commit message that is descriptive of the changes that you made. It is encouraged that you make commits
often so you can keep track of your changes more easily and avoid overwhelmingly large commits when you look back on your
version history.

When you are ready to move your local changes to a remote branch, you want to push to the correct branch
and potentially set the upstream if it does not yet exist:

```
git push -u origin <current_branch_name>
```

## Merging branches

There may be times where you want to merge two branches together, whether you diverged on some ideas and finally
want to synthesize them, or you just want to update your issue's branch with the main branch. In any case, merging
branches will be inevitable as part of the development process, so it is essential to understand how to merge branches.

=== "Merge Local Branch"

    ``` bash
    # Checkout to destination branch
    git checkout <dest_branch>

    # Merge with local copy of other branch
    git merge <other_branch>
    ```

=== "Merge Remote Branch"

    ``` bash
    # Checkout to destination branch
    git checkout <dest_branch>

    # Fetch from remote
    git fetch

    # Merge remote copy of other branch
    git merge origin/<other_branch>
    ```

!!! info

    Merging a remote branch into its local counterpart using the method above is essentially
    the same operation as `git pull`.

Once the merge operation is complete, your destination branch should have updates both from itself and the other
branch that you merge. If you do a `git log`, you will also see a new commit that indicates that the merge happened.

## Resolving merge conflicts

Merging two branches is not always easy since the commit history for both branches could look quite different, and
therefore conflicting changes can easily be made. If you run into a scenario like this, you may get something like this:

![image](../../../assets/images/github/workflow/merge_conflict.png)

Upon inspecting `bar.txt`, we see the following:

![image](../../../assets/images/github/workflow/merge_conflict_file.png)

Resolving merge conflicts is not always a trivial task, but there are many ways to resolve them which include:

- [Resolving on GitHub](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-on-github){target=_blank}
(recommended)
- [Resolving in Command Line](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/addressing-merge-conflicts/resolving-a-merge-conflict-using-the-command-line){target=_blank}

!!! tip

    If you cannot resolve a merge conflict on your own, reach out to your lead for help!
