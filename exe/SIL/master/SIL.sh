#!/bin/bash
set -e

SCRIPT_FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SIM_HOME_PATH=$(echo $SCRIPT_FILE_DIR | sed 's/\/exe\/SIL\/master//g')
S_DEFINE_PATH=$SCRIPT_FILE_DIR

if [ ! -f "/usr/local/include/hiredis/hiredis.h" ]; then
    echo "Insall the hiredis (client)"
    cd $SIM_HOME_PATH/third-party/hiredis
    make && sudo make install
fi

redis_pid_arr=( $(ps -aux | grep $USER | grep -E "redis-server\s\*\:[0-9]+" | awk '{print $2}') )
redis_pid_arr_len=${#redis_pid_arr[@]}
for name in "${redis_pid_arr[@]}"; do
    kill $redis_pid_arr
done

redis-server &
REDIS_SERVER_PID=$(echo $!)


cd $S_DEFINE_PATH
trick-CP
./S_main_Linux_*_x86_64.exe RUN_golden/golden_dm.cpp &

cd $SIM_HOME_PATH/exe/SIL/slave
trick-CP
./S_main_Linux_*_x86_64.exe RUN_golden/golden_fc.cpp

cd $S_DEFINE_PATH
python3 $SIM_HOME_PATH/utilities/generate_error.py $SIM_HOME_PATH/tables/golden_answer/golden.csv $S_DEFINE_PATH/RUN_golden/log_rocket_csv.csv -l
python3 $SIM_HOME_PATH/utilities/ci_test.py $S_DEFINE_PATH/result.csv 5e-5 | tee test_result
# Test the exit status of the command before pipe
kill $REDIS_SERVER_PID
test ${PIPESTATUS[0]} -eq 0
