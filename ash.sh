#!/bin/bash

sensor_pos=18
echo $sensor_pos

/opt/etherlab/bin/ethercat sla
echo 'write some shit to the sync manager at first to make the anybus chip happy'
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $sensor_pos 0x1c13 0 2
sleep 2

