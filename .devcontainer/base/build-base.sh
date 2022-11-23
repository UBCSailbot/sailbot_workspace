#!/bin/bash

git clone https://github.com/ompl/ompl.git
pushd ompl
git reset --hard d8375d842a7f016d64f27a81d6f95eaa83fbe595
popd

docker build --tag ros-ompl-jammy --file base.Dockerfile .

rm -rf ompl
