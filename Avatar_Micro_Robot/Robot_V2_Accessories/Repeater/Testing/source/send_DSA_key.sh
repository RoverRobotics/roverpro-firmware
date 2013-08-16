#!/bin/bash

sftp -o "StrictHostKeyChecking no" -q admin@192.168.88.1 << ENDSFTP
put DSA/id_dsa.pub /
ENDSFTP
ssh -o "StrictHostKeyChecking no" -q -t admin@192.168.88.1 << ENDSSH
/put Starting
/user ssh-keys import public-key-file=id_dsa.pub user=admin 
/put Finished
ENDSSH

