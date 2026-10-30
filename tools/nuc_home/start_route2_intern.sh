#!/usr/bin/env bash
# route2_intern bring-up from whatever deploy is current (~/use_deploy.sh).
#   ~/start_route2_intern.sh            # D5_to_A
# ~/start_wheelchair_localization.sh starts the same map and route; this one
# prints the route banner and takes the route key.
set -eo pipefail
exec bash /home/mprp3/wheelchair_deploys/current/source-linux/tools/start_route2_intern.sh "$@"
