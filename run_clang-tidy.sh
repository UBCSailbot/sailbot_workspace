files=$(find ./src/network_systems/ -type f \( -iname \*.cpp -o -iname \*.h \))

if [[ ! -z ${files} ]]; then
    clang-tidy -p ./build/ --format-style=file $files
fi
