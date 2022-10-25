FILES=$(find ./src/network_systems/ -type f \( -iname \*.cpp -o -iname \*.h \) -not -path "./src/network_systems/lib/**")

clang-tidy-10 --format-style=file $FILES
