#!/bin/bash

ssh -o "StrictHostKeyChecking no" -q -i DSA/id_dsa admin@192.168.88.1 << ENDSSH
/system reset-configuration
ENDSSH
}