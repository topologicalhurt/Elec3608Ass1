#!/bin/bash

test_results=$(
  tail -n +1 -v tests/test*.log |
  awk '
    /^==> / { fn=$2; sub(/^tests\//,"",fn); sub(/\.log$/,"",fn); next }
    match($0, /Simulated [0-9]+ cycles/) { print fn ", " substr($0, RSTART, RLENGTH) }
  ' |
  sort -t',' -k1,1V
)

echo "=== Test Results Summary: ==="
echo "$test_results"
