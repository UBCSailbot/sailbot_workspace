# Differences Between C and C++

For most use cases, you can think of C++ as a superset of C. While this is not technically true, more often than not
you are able to write standard C code for a C++ program without issues. However, doing so ignores a lot of the benefits
and reasons to use C++.

## Classes and Structs

In C structs can only contain member variables, but in C++ structs are basically classes but with a default member
visibility of public instead of private.

???+ example
    The following code blocks are equivalent.

    ```C++
    struct foo {
    private:
        int x;
        void helper(void);
    public:
        foo(int y);
    }
    ```

    ```C++
    class foo {
    private:
        int x;
        void helper(void);
    public:
        foo(int y);
    }
    ```

## Namespaces

One problem that is prevalent in C concerns the scoping of names. For example, let there be two files `A.h` and `B.h`
and a program `ighxy.c`, and let them both contain a `float x` and `int bar(void)`.

Our program cannot compile because the linker cannot distinguish which `bar()` function we want to use! One way to fix
this in a C program would be to rename them `a_bar()` and `b_bar()`. Although this fix seems trivial for this example,
applying it to a file that has potentially 100 functions can be a lot more difficult, especially if two files just
happen to share the same prefix for their functions!

C++ introduces namespaces to tackle this problem. With namespaces, we can deal with naming conflicts much more easily.
Though be aware that namespaces are not necessary everywhere. See the following code snippet to see how they work.

??? Example

    === "C"

        ```C title="A.h"
        float x;
        int bar(void);
        ```

        ```C title="B.h"
        float x;
        int bar(void);
        ```

        ```C title="ighxy.c"
        #include "A.h"
        #include "B.h"

        int main(void) {
            int a = bar();
            ...
        }
        /* Error, does not compile*/
        ```

    === "C++"

        ```C++ title="A.h"
        namespace a {
        float x;
        int bar(void);
        }
        ```

        ```C++ title="B.h"
        namespace b {
        float x;
        int bar(void);
        }
        ```

        ```C++ title="ighxy.cpp"
        #include "A.h"
        #include "B.h"

        int main(void) {
            int a = a::bar();
            int b = b::bar();
            float xa = a::x;
            float xb = b::x;
            /* No problem! */
            ...
        }
        ```

???+ warning
    You may come across something like:

    ```C++ title="example.cpp"
    using namespace std;
    namespace io = std::filesystem;

    int main(int argc, char* argv[]) {
        bool isDirectory = io::is_directory(argv[1]);  // Equivalent to std::filesystem::is_directory(argv[1])
        cout << isDirectory << endl;
        return 0;
    }
    ```

    There are two things going on here.

    First, `using namespace std` makes all functions and types defined within the standard namespace and included via 
    `#include` directives visible to `example.cpp`. If you are familiar with Python, the Python equivalent of this would
    be `import std as *`. However, it is considered *bad* practice to do this as it eliminates the point of using
    namespaces.

    === "OK"

        ```C++
            class string {
                // Insert implementation here
            }

            int main(void) {
                string ourString = "Our own string implementation";
                std::string stdString = "Standard Library string implementation";
                ...
            }
        ```

    === "Not OK"

        ```C++
            using namespace std;

            // ERROR - multiple definitions of type string
            class string {

            }
        ```

        The compiler cannot infer which implementation we want.

    Secondly, `namespace io = std::filesystem` is basically an alias for the `std::filesystem` namespace. This practice
    is acceptable for long namespace identifiers, but be careful as it can still run into namespace conflicts if your
    alias is the same as another namespace or alias.

## Constant Expressions

In C, if we want to declare a constant or a function/expression that we want to be evaluated at compile time, we need
to use `#define` statements. One of the problems with `#define` statements is that they perform a simple copy paste
wherever they're used. For example:

=== "Before Precompile"

    ```C
    #define PI 3.14F
    #define AREA_OF_CIRCLE(radius) ((PI) * (radius) * (radius))

    int main(void) {
        float area = AREA_OF_CIRCLE(2.5F);
        ...
    }
    ```

=== "After Precompile"

    ```C++
    int main(void) {
        float area = ((3.14F) * (2.5F) * (2.5F));
        ...
    }

    ```

!!! note
    `AREA_OF_CIRCLE` is a macro with arguments. If you are confused by it, [this resource](https://www.cs.yale.edu/homes/aspnes/pinewiki/C(2f)Macros.html){target=_blank}
    has a detailed explanation on how they work.

Because of this copy-pasting, you need to be very careful with syntax, sometimes necessitating an ugly
[`do {} while(0)` wrapper](https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block){target=_blank}.
Moreover, symbols declared with `#define` are always globally visible, ignoring namespaces!

In C++, the use of constant expressions are preferred.

```C++
constexpr float pi = 3.14F;
constexpr float area_of_circle(float radius) {
    return pi * radius * radius;
}
```

Constant expressions do *not* get copy pasted, and are instead placed in program memory just like a normal variable
or function. They also respect namespaces and function scopes, meaning the following code compiles.

```C++ title="Constant Expression Scoping"
void foo(void) {
    constexpr float rand = 123.456;
    ...
}

void bar (void) {
    constexpr float rand = 789.123;
    ...
}
```

## Lambdas

Lambdas are primarily useful when you need to register a callback function one time and don't feel it's necessary to
write out a full function. They are in no way required though, so do not worry about learning them. However, it's
necessary to know that they exist such that you don't get confused when reading code. For more information, [go here](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170){target=_blank}
for Microsoft's explanation.

## Misc

### Arrays

Using the [C++ implementation of arrays](https://en.cppreference.com/w/cpp/container/array#:~:text=std%3A%3Aarray%20is%\20a%20container%20that%20encapsulates%20fixed%20size,C-style%20array%2C%20it%20doesn%27t%20decay%20to%20T%2A%20automatically.){target=_blank}
is preferred over C arrays. It is simply easier and safer to work with than a standard C array without any performance
costs.

???+ example

    Passing an array to a function an iterating over it

    === "C"

        ```C
        #include "stdio.h"

        void print_contents(int *arr, int size) {
            for (int i = 0; i < size; i++) {
                printf("%d\n", *arr);
            }
        }

        int main(void) {
            int arr[5] = {0, 1, 2, 3, 4};
            foo(arr, 5);
            return 0;
        }
        ```
        We can't even guarantee that the integer pointer `arr` is an array!

    === "C++"

        C++ 20 makes passing arrays around a lot simpler. Do not worry about understanding the code shown below. It uses
        some fairly advanced concepts and exists to illustrate how different such a simple operation can be.

        ```C++
        #include <iostream>
        #include <array>
        #include <span>

        void print_contents(std::span<int> container) {
            for (const auto &e : container) {
                std::cout << e << std::endl;
            }
        }

        int main(void) {
            std::array<int, 5> arr = {0, 1, 2, 3, 4};
            foo(arr);
            return 0;
        }
        ```

        The advantages of the C++ version are:

        * Size is implicitly part of the object
        * We guarantee that foo takes a container, but it does not care if it's an array or, say, a vector, which is
          preferable in this scenario where we simply iterate through the container's existing elements
