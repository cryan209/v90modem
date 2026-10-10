# Source this: find or stop OUR sip_v90_modem, never someone else's.
#
# A rig host can run servers that are not ours to kill.  Since 2026-10-07 the
# v90modem-sip container on tower has run a long-lived courier-x2 service
# (ext 6012, --local-port 5072), and `pkill -x sip_v90_modem` takes it down
# with the server under test.  A server is identified here by its SIP port,
# the one resource two servers cannot share anyway: `--local-port N` on its
# command line, or 5060 (sip_modem.c's default) when the flag is absent.
# Arm binaries such as sip_v90_modem.fixedpt count as servers too.
#
#   . "$SP/rig_server.sh"
#   rig_server_pids 5060              # PIDs on this host, one per line
#   rig_server_kill 5060              # SIGTERM them
#   docker exec C sh -c "$(rig_server_kill_cmd 5074); ..."   # same, in a container
#
# The *_cmd forms print plain POSIX sh (ps + awk, no /proc), so they run the
# same on macOS, on tower and inside the containers.  Where a script starts the
# server itself with `&`, prefer killing $! (as v90_notch_ab.sh does); these
# are for servers started by `docker exec -d` or left by an earlier run.

rig_server_pids_cmd() {
    printf '%s' "ps -eo pid=,args= | awk -v want=$1 '{ n = split(\$2, a, \"/\"); if (a[n] !~ /^sip_v90_modem(\\..*)?\$/) next; port = 5060; for (i = 3; i < NF; i++) if (\$i == \"--local-port\") port = \$(i + 1); if (port == want) print \$1 }'"
}

rig_server_kill_cmd() {
    printf '%s' "for p in \$($(rig_server_pids_cmd "$1")); do kill \$p; done"
}

rig_server_pids() { sh -c "$(rig_server_pids_cmd "$1")"; }
rig_server_kill() { sh -c "$(rig_server_kill_cmd "$1")"; }
