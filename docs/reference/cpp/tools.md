# Tools

A lot goes into making a well structured C++ project, much more than any one team should have to do.

## CMake

CMake is a powerfull build automation tool that makes compiling code for large projects with a lot of interoperating
files a lot easier. Steps 1-3 of the [official tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html){target=_blank}
are great for understanding the basics.

## GDB

The [GNU Project Debugger](https://www.sourceware.org/gdb/){target=_blank} is the most commonly debugger for the C
language family.
VSCode also has a degree of integration with GDB that allows an easy to use GUI. This [GDB cheat sheet](https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf){target=_blank}
has all the GDB comands you will need to know. Be aware the VSCode has GUI buttons for some of these commands that are
easier to use.

<!-- TODO Add examples with screenshots -->

## GoogleTest

[GoogleTest](https://github.com/google/googletest){target=_blank} is the C++ unit testing framework we will be using.
The [GoogleTest Primer](https://google.github.io/googletest/primer.html){target=_blank} is a good place to start.

??? example

    === "Cached Fibonacci Program"

        ```C++ title="cached_fib.h"
        #include <vector>
        class CachedFib {
        public:
            void CachedFib(int n);
            int  getFib(int n);
        private:
            std::vector<int> cache;
        }
        ```

        ```C++ title="cached_fib.cpp"
        #include <iostream>
        #include <vector>
        #include "cached_fib.h"

        void CachedFib::CachedFib(int n) {
            cache.push_back(0);
            cache.push_back(1);
            for (int i = 2; i < n; i++) {
                cache.push_back(cache[i - 1] + cache[i - 2]);
            }
        }

        int CachedFib::getFib(int n) {
            if (cache.size() < n) {
                for (int i = cache.size(); i < n; i++) {
                    cache.push_back(cache[i-1] + cache[i-2]);
                }
            }
            std::cout << cache[n - 1] << std::endl;
        }
        ```

    === "Test Cached Fibonacci Program"

        ```C++ title="test_cached_fib.cpp"

        #include "cached_fib.h"
        #include "gtest/gtest.h"

        CachedFib::testFib;

        class TestFib : public ::testing::Test {
        protected:
            void Setup override {
                // Every time a test is started, testFib is reinitialized with a constructor parameter of 5
                testFib = CachedFib(5);
            }
        }

        TEST_F(TestFib, TestBasic) {
            ASSERT_EQ(getFib(5), 3) << "5th fibonacci number must be 3!";
        }

        // more tests

        ```

<!-- ## Google Mock Not sure if we're going to use this yet -->

## Google Protocol Buffer

[Google Protocol Buffer](https://developers.google.com/protocol-buffers){target=_blank} (Protobuf) is a portable data serialization
method. We use it over other methods like JSON and XML because it produces smaller binaries, an important consideration
when sending data across an ocean. Unfortunately, there does not seem to be a easy to follow tutorial for using them,
but here are the [C++ basics](https://developers.google.com/protocol-buffers/docs/cpptutorial){target=_blank}. The page
is quite dense and can be hard to follow, so do not worry if you do not understand it.

## Clang

In its most basic form, [Clang](https://clang.llvm.org/){target=_blank} is a compiler for the C language family.
Clang has multiple
benefits like easier portability compared to, for example, GCC. Clang is actually "half" the compiler, the other half
being LLVM. Without going into unnecessary detail, Clang compiles C++ code to a generic language before LLVM compiles
it to machine specific language.

### Clangd

[Clangd](https://clangd.llvm.org/){target=_blank} is the Clang language server. It provides a much more powerful
intellisense than the default one used in VSCode's C/C++ extension.

### Clang-Tidy

[Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/){target=_blank} is a linting tool, who's main purpose is to catch potential
programming errors caused by bad programming style/practices using just static analysis.

### Clang Format

An autoformatting tool that makes enforcing style guidelines much easier. When se tup, it corrects formatting as soon
as you hit save.

## llvm-cov

We will use [llvm-cov](https://llvm.org/docs/CommandGuide/llvm-cov.html){target=_blank} to evaluate our test coverage.
When used with
[genhtml](https://www.systutorials.com/docs/linux/man/1-genhtml/){target=_blank}, we can generate HTML reports that that
show our line, function, and branch coverage line-by-line.
