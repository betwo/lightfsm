#!/bin/bash -i
set -euo pipefail
set -x
echo "running test coverage"

THISDIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# build
catkin profile list | grep coverage || {
    catkin profile add coverage
    catkin profile set coverage
    catkin config \
        --build-space "build_coverage" \
        --devel-space "devel_coverage" \
        --cmake-args -DCMAKE_C_FLAGS='--coverage' -DCMAKE_CXX_FLAGS='--coverage'
}

catkin profile set coverage
catkin build sbc15_fsm
catkin test sbc15_fsm

BUILD=$(catkin locate -b)
ROOT=$(echo ${BUILD}/.. | xargs realpath)
DIR=$THISDIR/traces
cd $ROOT

# clean
cd $THISDIR
rm -fr coverage
rm -fr $DIR
rm *.info || true

# run tests
cd $THISDIR
mkdir $DIR

set +e
find ${BUILD} -iname *.gcno | xargs cp -t $DIR
set -e

lcov --directory $DIR --zerocounters -q

cd ${BUILD}/sbc15_fsm
make run_tests

cd $THISDIR
find ${BUILD} -iname *.gcda | xargs mv -t $DIR
lcov --capture --directory $DIR --output-file $THISDIR/test.info
lcov --remove test.info           "/usr*" -o test_extracted.info
lcov --remove test_extracted.info "/opt*" -o test_extracted.info

# output
cd $THISDIR
genhtml --demangle-cpp --output-directory $THISDIR/coverage test_extracted.info
xdg-open coverage/index.html

catkin profile set default