#!/bin/bash

spos=$(/opt/etherlab/bin/ethercat sla | grep Compact | grep -Po '\d+\s+\d+:\K(\d+)')
echo 'The FSR is found at' $spos

echo 'write some shit to the sync manager at first to make the anybus chip happy'
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $spos 0x1c13 0 2
sleep 2

echo 'clear SM3 settings'
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $spos 0x1c13 0 0

subindex=1
for pdo_entries in 0x1a01 ; do
	echo 'set sm pdos entries'
	echo $pdo_entries
	echo $subindex

	/opt/etherlab/bin/ethercat state INIT
	echo 'INITED'
	sleep 2
	/opt/etherlab/bin/ethercat state PREOP
	echo 'PREOPED'
	sleep 2
	/opt/etherlab/bin/ethercat download -a 0 -t uint16 -p $spos 0x1c13 $subindex $pdo_entries
	sleep 2
	let "subindex=$subindex+1"
done

echo 'finish sync manager settings'
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $spos 0x1c13 0 1
#
#

echo 'clear SM2 settings'
sleep 2
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $spos 0x1c12 0 0

echo 'setup SM2 pdo entries'
sleep 2
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint16 -p $spos 0x1c12 2 0x1601

echo 'finish SM2 settings'
sleep 2
/opt/etherlab/bin/ethercat state INIT
echo 'INITED'
sleep 2
/opt/etherlab/bin/ethercat state PREOP
echo 'PREOPED'
sleep 2
/opt/etherlab/bin/ethercat download -a 0 -t uint8 -p $spos 0x1c12 0 2

