#!/bin/bash
instances=`ps aux | grep "./timeout" | grep -v grep | wc -l`
echo $instances
for i in $(seq 1 1 1)
do
  echo "Welcome $i times"
  for (( ; ; ))
  do
    sleep 5
    instances=`ps aux | grep "./timeout" | grep -v grep | wc -l`
    echo "instances:$instances"
    if [ "$instances" -lt "4" ]
    then
      break       	   #Abandon the loop.
    sleep 5
    fi
  done
    if [ "$i" -lt "31" ]
    then
#echo "skipping"
    if [ "$i" -lt "10" ]
    then
      nohup ./timeout -t 1800 ./planRAStar sequential/openstacks-strips/p0$((i))-domain.pddl sequential/openstacks-strips/p0$((i)).pddl plan_RA_51h_greedy_earlyterm_openstack_0$i >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_OPENSTACKS_$((i)) &
    else
      nohup ./timeout -t 1800 ./planRAStar sequential/openstacks-strips/p$((i))-domain.pddl sequential/openstacks-strips/p$((i)).pddl plan_RA_51h_greedy_earlyterm_openstack_$i >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_OPENSTACKS_$((i)) &
    fi
    elif [ "$i" -lt "61" ]
    then
    if [ "$i" -lt "40" ]
    then
      nohup ./timeout -t 1800 ./planRAStar sequential/parcprinter-strips/p0$((i-30))-domain.pddl sequential/parcprinter-strips/p0$((i-30)).pddl plan_RA_51h_greedy_earlyterm_parcprinter_0$((i-30))  >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_PARCPRINTER_$((i-30)) &
    else
      nohup ./timeout -t 1800 ./planRAStar sequential/parcprinter-strips/p$((i-30))-domain.pddl sequential/parcprinter-strips/p$((i-30)).pddl plan_RA_51h_greedy_earlyterm_parcprinter_$((i-30))  >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_PARCPRINTER_$((i-30)) &
    fi
    elif [ "$i" -lt "91" ]
    then
      if [ "$i" -lt "70" ]
      then
	nohup ./timeout -t 1800 ./planRAStar sequential/pegsol-strips/p0$((i-60))-domain.pddl sequential/pegsol-strips/p0$((i-60)).pddl plan_RA_51h_greedy_earlyterm_pegsol_0$((i-60))  >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_PEGSOL_$((i-60)) &
      else
	nohup ./timeout -t 1800 ./planRAStar sequential/pegsol-strips/p$((i-60))-domain.pddl sequential/pegsol-strips/p$((i-60)).pddl plan_RA_51h_greedy_earlyterm_pegsol_$((i-60))  >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_PEGSOL_$((i-60)) &
      fi
    elif [ "$i" -lt "121" ]
    then
    if [ "$i" -lt "100" ]
    then
      nohup ./timeout -t 1800 ./planRAStar sequential/scanalyzer-strips/p0$((i-90))-domain.pddl sequential/scanalyzer-strips/p0$((i-90)).pddl plan_RA_51h_greedy_earlyterm_scanalyzer_0$((i-90))  >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_SCANALYZER_$((i-90)) &
    else
      nohup ./timeout -t 1800 ./planRAStar sequential/scanalyzer-strips/p$((i-90))-domain.pddl sequential/scanalyzer-strips/p$((i-90)).pddl plan_RA_51h_greedy_earlyterm_scanalyzer_$((i-90)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_SCANALYZER_$((i-90)) &
    fi
    elif [ "$i" -lt "151" ]
    then
      if [ "$i" -lt "130" ]
      then
	nohup ./timeout -t 1800 ./planRAStar sequential/sokoban-strips/p0$((i-120))-domain.pddl sequential/sokoban-strips/p0$((i-120)).pddl plan_RA_51h_greedy_earlyterm_sokoban_p0$((i-120)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_SOKOBAN_$((i-120)) &
      else
	nohup ./timeout -t 1800 ./planRAStar sequential/sokoban-strips/p$((i-120))-domain.pddl sequential/sokoban-strips/p$((i-120)).pddl plan_RA_51h_greedy_earlyterm_sokoban_p$((i-120)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_SOKOBAN_$((i-120)) &
      fi
    elif [ "$i" -lt "181" ]
    then
      if [ "$i" -lt "160" ]
      then
	nohup ./timeout -t 1800 ./planRAStar sequential/transport-strips/p0$((i-150))-domain.pddl sequential/transport-strips/p0$((i-150)).pddl plan_RA_51h_greedy_earlyterm_transport_p0$((i-150)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_TRANSPORT_$((i-150)) &
      else
	nohup ./timeout -t 1800 ./planRAStar sequential/transport-strips/p$((i-150))-domain.pddl sequential/transport-strips/p$((i-150)).pddl plan_RA_51h_greedy_earlyterm_transport_p$((i-150)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_TRANSPORT_$((i-150)) &
      fi
    elif [ "$i" -lt "211" ]
    then
      if [ "$i" -lt "190" ]
      then
	nohup ./timeout -t 1800 ./planRAStar sequential/woodworking-strips/p0$((i-180))-domain.pddl sequential/woodworking-strips/p0$((i-180)).pddl plan_RA_51h_greedy_earlyterm_woodworking_p0$((i-180)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_WOODWORKING_$((i-180)) &
      else
	nohup ./timeout -t 1800 ./planRAStar sequential/woodworking-strips/p$((i-180))-domain.pddl sequential/woodworking-strips/p$((i-180)).pddl plan_RA_51h_greedy_earlyterm_woodworking_p$((i-180)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_WOODWORKING_$((i-180)) &
      fi
    elif [ "$i" -lt "231" ]
    then
	nohup ./timeout -t 1800 ./planRAStar sequential/tsp-neg-prec/domain.pddl sequential/tsp-neg-prec/p$((i-210)).pddl plan_RA_51h_greedy_earlyterm_tsp_p$((i-210)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_TSP_$((i-210)) &
    elif [ "$i" -lt "261" ]
    then
      nohup ./timeout -t 1800 ./planRAStar sequential/m10-cond-eff/domain.pddl sequential/m10-cond-eff/p$((i-231)).pddl plan_RA_51h_greedy_earlyterm_m10-cond-eff_p$((i-231)) >& /home/santiago/FD_RIDA/logs_demo_IPC2013/log_RA_51h_32bits_NewDemote2ndSub_M10_$((i-231)) &
     fi
done
