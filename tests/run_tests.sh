#!/bin/sh
# Compile et exécute les tests du suivi de blobs sur PC.
#   ./tests/run_tests.sh          scénarios unitaires
#   ./tests/run_tests.sh -v       idem, avec le détail image par image
#   ./tests/run_tests.sh --fuzz   test de robustesse (scénarios aléatoires)
set -e
cd "$(dirname "$0")/.."
mkdir -p build
if [ "$1" = "--fuzz" ]; then
  g++ -std=c++17 -O2 -Wall -Wextra -Icomponents/blobsense \
    tests/fuzz_tracker.cpp components/blobsense/tracker.cpp -o build/fuzz_tracker
  ./build/fuzz_tracker
else
  g++ -std=c++17 -O1 -Wall -Wextra -Icomponents/blobsense \
    tests/test_tracker.cpp components/blobsense/tracker.cpp -o build/test_tracker
  ./build/test_tracker "$@"
fi
