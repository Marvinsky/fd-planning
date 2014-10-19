#!/bin/bash
instances=`ps aux | grep "./downward" | grep -v grep | wc -l`
echo $instances
for i in $(seq 1 1 280)
do
  echo "Welcome $i times"
  for (( ; ; ))
  do
    sleep 5
    instances=`ps aux | grep "./downward" | grep -v grep | wc -l`
    echo "instances:$instances"
    if [ "$instances" -lt "4" ]
    then
      break       	   #Abandon the loop.
    sleep 5
    fi
  done
    if [ "$i" -lt "21" ]
    then
#echo "skipping"
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/TRANSPORT-$i >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_TRANSPORT_$i &
    elif [ "$i" -lt "41" ]
    then
    nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/SCANALYZER-$((i-20)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_SCANALYZER_$((i-20)) &
echo "skipping"
    elif [ "$i" -lt "61" ]
    then
echo "skipping"
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/TIDYBOT-$((i-40)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_TIDYBOT_$((i-40)) &
    elif [ "$i" -lt "81" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/WOODWORKING-$((i-60)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_WOODWORKING-$((i-60)) &
    elif [ "$i" -lt "101" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/ELEVATORS-$((i-80)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_ELEVATORS-$((i-80)) &
    elif [ "$i" -lt "121" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/PEGSOL-$((i-100)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_PEGSOL_$((i-100)) &
    elif [ "$i" -lt "141" ]
    then
#echo "skipping"
    nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/SOKOBAN-$((i-120)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_SOKOBAN_$((i-120)) &
    elif [ "$i" -lt "161" ]
    then
echo "skipping"
    nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/NOMYSTERY-$((i-140)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_NOMYSTERY_$((i-140)) &
    elif [ "$i" -lt "181" ]
    then
    nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/FLOORTILE-$((i-160)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_FLOORTILE$((i-160)) &
    elif [ "$i" -lt "201" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/VISITALL-$((i-180)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_VISITALL_$((i-180)) &
    elif [ "$i" -lt "221" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/PARCPRINTER-$((i-200)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_PARCPRINTER_$((i-200)) &
    elif [ "$i" -lt "241" ]
    then
    nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/BARMAN-$((i-220)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_BARMAN_$((i-220)) &
    elif [ "$i" -lt "261" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/PARKING-$((i-240)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_PARKING_$((i-240)) &
    elif [ "$i" -lt "281" ]
    then
      nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/OPENSTACKS-$((i-260)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_OPENSTACKS_$((i-260)) &
#nohup ./timeout -m 4194304 -t 1800 ./FD_Loc_Inc_LMCUT.sh FD_problems/FLOORTILE-$((i-20)) >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_Loc_Inc_LMCUT_FLOORTILE_$((i-20)) &
     fi
#./downward-1 --search "astar(lmcut())" --random-seed 1 --Degree 1 --Phase SOLVING --sampled-Hoff-Trees 0 < FD_problems/FLOORTILE-$i >&& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_test_$i &
#    ./downward-1  --random-seed 1 --Degree 1 --Phase SOLVING --sampled-Hoff-Trees 0 < FD_problems/FLOORTILE-$i >& /home/santiago/FD_RIDA/logs_prep_IPC2013/log_test_$i
done
