#!/bin/bash -i
echo "running test coverage"

THISDIR=$(pwd)
ROOT=$(roscd; cd ../; pwd)
DIR=$THISDIR/traces

# build
cd $ROOT
catkin_make
catkin_make tests

# clean
cd $THISDIR
rm -fr coverage
rm -fr $DIR
rm *.info

# run tests
cd $THISDIR
mkdir $DIR
find $ROOT/build -iname *.gcno | xargs cp -t $DIR

lcov --directory $DIR --zerocounters -q

cd $ROOT/build
#make run_tests
make run_tests_sick14_state_machine_gtest

cd $THISDIR
find $ROOT/build -iname *.gcda | xargs mv -t $DIR
lcov --capture --directory $DIR --output-file $THISDIR/test.info
lcov --remove test.info           "/usr*" -o test_extracted.info
lcov --remove test_extracted.info "/opt*" -o test_extracted.info

# output
cd $THISDIR
genhtml --demangle-cpp --output-directory $THISDIR/coverage test_extracted.info
gnome-open coverage/index.html
