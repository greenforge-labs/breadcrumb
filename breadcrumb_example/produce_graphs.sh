#! /bin/bash

breadcrumb launch/full_system.launch.py -o ./ros_graphs/graph.dot  --graph-type grouped_and_full_system
for f in ./ros_graphs/*.dot; do
    dot -Tsvg "$f" -o "${f%.dot}.svg"
done
