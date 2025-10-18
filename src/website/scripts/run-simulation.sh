#!/bin/bash

[ ! -d "venv" ] && python3 -m venv venv

source venv/bin/activate
pip install pymongo --quiet

cd tests/simulation
python3 simulation.py