files=$(find ./src/network_systems/ -type f \( -iname \*.cpp -o -iname \*.h \) -not -path "./src/network_systems/lib/googletest/**")

clang-tidy-10 --format-style=file $files
