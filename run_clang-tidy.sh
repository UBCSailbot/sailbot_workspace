files=$(find ./src/network_systems/ -type f \( -iname \*.cpp -o -iname \*.h \))

clang-tidy-10 -p ./build/ --format-style=file $files
