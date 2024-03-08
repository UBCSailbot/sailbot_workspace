# Conventions

At UBC Sailbot, we follow standards in how we code to maintain a clean and comprehensible codebase.
This page addresses what conventions we use specifically when programming in Python and the tools to help us maintain
these conventions.

## Style guide

### Linting

To ensure that the codebase stays clean, we use [flake8](https://flake8.pycqa.org/en/5.0.4/#){target=_blank}, which is a
tool for style guide enforcement mostly based off [pep8](https://peps.python.org/pep-0008/){target=_blank}. To automate
most of this process, we use [autopep8](https://github.com/hhatto/autopep8){target=_blank}, which is a tool that resolves
most style issues. However, there will be some issues that must be resolved by you!

Refer to this [guide](https://realpython.com/python-pep8/){target=_blank} on how to write readable code in python with the
pep8 style guide.

!!! note

    Our CI automatically checks that your code follows the pep8 standard. If it does not, your pull requests will
    be blocked from being merged until those issues are resolved!

### Type hinting

Even though Python is a dynamically typed language, newer versions support type hinting. Type hinting catches
errors, documents code, improves IDEs and linters, and helps build and maintain a clean software architecture.[^1]
Expanding on how it catches errors, a static type checker such as [`mypy`](https://mypy.readthedocs.io/en/stable/index.html){target=_blank}
can be used.

There is some syntax to get familiar in order to use type checking. We recommend the following resources:

- [mypy Typing Cheatsheet](https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html){target=_blank}
- [PEP 483: The Theory of Type Hints (A Simplified Guide)](https://peps.python.org/pep-0483/){target=_blank}
- [PEP 484: Type Hints (Fully Comprehensive Guide)](https://peps.python.org/pep-0484/){target=_blank}

Below are a few examples of using type hinting:

???+ example "Return the sum of a sequence"

    ```python
    from typing import Sequence, Union


    Number = Union[int, float]


    def sumseq(seq : Sequence[Number]) -> Number:
        return sum(seq)
    ```

???+ example "Function with optional parameters and default values"

    ```python
    from typing import Optional


    def printArgs(a : str, b : str="World", c : Optional[str]=None) -> None:
        print(f"Value of a: {a}")
        print(f"Value of b: {b}")
        if c is not None:
            print(f"Value of c: {c}")
    ```

??? example "Function with custom class"

    ```python
    class MyClass:
        def __init__(self) -> None:
            pass


    def foo(a : MyClass) -> None:
        print(a)
    ```

??? example "Forward referencing a class"

    === "With `__future__`"

        ```python
        from __future__ import annotations


        def foo(a : MyClass) -> None:
            print(a)


        class MyClass:
            def __init__(self) -> None:
                pass
        ```

    === "Without `__future__`"

        ```python
        def foo(a : 'MyClass') -> None:
            print(a)


        class MyClass:
            def __init__(self) -> None:
                pass
        ```

??? example "Function that never returns"

    ```python
    from typing import NoReturn


    def bar() -> NoReturn:
        while True:
            print("Hello World!")
    ```

## Documentation

Code is written once and read a thousand times, so it is important to provide good documentation for current
and future members of the software team. The major things that we document in our code are:

1. **Classes and Objects:**
    - What does it represent? What is it used for?
    - What are its member variables? What are they used for?
2. **Functions:**
    - What are the inputs and outputs?
    - What is the overall behavior and purpose of the function?
3. **Code:**
    - Is a line of code obscure and/or not clear? Add an inline comment to clear things up.
    - Break down a large process.

Ideally, the third point should be avoided as much as possible since we would want our code to be
self explanatory. It should be done only when absolutely necessary.

### Generating docstrings

We use a vscode extension called [autoDocstring](https://marketplace.visualstudio.com/items?itemName=njpwerner.autodocstring){target=_blank}
which autogenerates docstrings that we use to document our code. To install this extension, go to the `Extensions` tab in
vscode and search `autoDocstring` in the marketplace.

To generate docstrings, type `"""` at the beginning of the function that you want to document and the template
will be generated for you! If you use [type hinting](#type-hinting), this extention will autofill some of the
documentation for you!

!!! note

    The autoDocstring extension only works for functions. It does not work for classes and objects, so documenting these
    will have to be done manually. Be sure to follow the same format used by functions.

### Example on documentation

It's hard to imagine what good documentation looks like. We provide a few examples below of documenting code using the
autoDocstring extension. The extension uses [Google style docstrings](https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings){target=_blank}
by default.

???+ example "Documentation example on a function"

    ```python
    from typing import List
    def inner_product(v1 : List[float], v2 : List[float]) -> float:
        """
        Computes the inner product between two 1D real vectors. Input vectors should have the
        same dimensions.

        Args:
            v1 (List[float]): The first vector of real numbers.
            v2 (List[float]): The second vector of real numbers.

        Returns:
            float : The inner product between v1 and v2
        """
        assert (len(v1) == len(v2)), "Input lists must have same length"

        # Iterate through elementwise pairs
        summation = 0
        for e1, e2 in zip(v1, v2):
            summation += (e1 * e2)
        return float(summation)
    ```

??? example "Documentation example with a stack"

    ```python
    from typing import Any
    class Stack:

        """
        This class represents a stack, which is an abstract data type that serves as a collection of
        elements. The stack is a LIFO datastructure defined by two main operations: Push and Pop.

        Attributes:
            __stack (List[Any]): A list containing the elements on the stack.
        """

        def __init__(self):
            """
            Initializes the Stack object.
            """
            self.__stack = []

        def push(self, element : Any) -> Any:
            """
            Pushes an element to the top of the stack.

            Args:
                element (Any): The element to be pushed on to the stack.
            """
            self.__stack.append(element)

        def pop(self) -> Any:
            """
            Removes the element at the top of the stack and returns it. If the stack is empty,
            then None is returned.

            Returns:
                Any, NoneType: The element at the top of the stack.
            """
            if self.is_empty():
                return None
            else:
                return self.__stack.pop()

        def is_empty(self) -> bool:
            """
            Determines whether the stack is empty or not.

            Returns:
                bool: Returns True if the stack is empty, and False otherwise.
            """
            empty = (len(self.__stack) == 0)
            return empty

        def __len__(self) -> int:
            """
            Gets the number of elements on the stack.

            Returns:
                int: The number of elements on the stack.
            """
            length = len(self.__stack)
            return length
    ```

For more examples, see [Example Google Style Python Docstrings](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html){target=_blank}.

[^1]: <https://realpython.com/lessons/pros-and-cons-type-hints/>
