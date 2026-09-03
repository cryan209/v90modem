#!/usr/bin/env python3
"""Score an ME_V8_NO_CI A/B: per call, what V.8 did at each end."""
import sys,os,re,glob
out=sys.argv[1]
rows=[]
for run in sorted(glob.glob(os.path.join(out,"*-r*"))):
    arm=os.path.basename(run).split("-")[0]
    cl=os.path.join(run,"coupler.log"); sl=os.path.join(run,"server.log")
    c=open(cl).read() if os.path.exists(cl) else ""
    # A run's directory exists from the moment it starts.  Score only calls
    # that reached the coupler's terminal summary, or an unfinished call is
    # reported as a failure that has not happened yet.
    if "rx packet lengths:" not in c:
        continue
    s=open(sl).read() if os.path.exists(sl) else ""
    m=re.search(r"\[TRACE \+(\d+)ms\] V8 result: status=([^(]+)\((\d+)\)",c)
    v8ms,v8st=(int(m.group(1)),m.group(2).strip()) if m else (None,"none")
    train=re.search(r"\[TRACE \+(\d+)ms\] enter TRAINING: mod=(\w+)",c)
    retry = s.count("failed before CM")
    srvtrain = "state=TRAINING" in s
    ci = c.count("<CI:")
    if "Incoming call" not in s:
        # The dial never reached the PBX -- a rig miss, not an arm result.
        v8st = "NO CALL"
    rows.append((os.path.basename(run),arm,v8ms,v8st,
                 (int(train.group(1)),train.group(2)) if train else None,
                 retry,srvtrain))
print("%-10s %-5s %9s  %-34s %-16s %6s %s"%("run","arm","v8_ms","v8 status","analogue TRAINING","srvRetry","srvTRAIN"))
for r in rows:
    print("%-10s %-5s %9s  %-34s %-16s %6d %s"%(r[0],r[1],r[2],r[3],
          ("%dms %s"%r[4]) if r[4] else "-",r[5],r[6]))
for arm in ("noci","ci"):
    sel=[r for r in rows if r[1]==arm]
    calls=[r for r in sel if r[3]!="NO CALL"]
    ok=sum(1 for r in calls if "successful" in r[3])
    ret=sum(r[5] for r in sel)
    print("%-5s: V.8 successful %d/%d calls placed (%d runs, %d no-call), "
          "answerer CM-wait timeouts %d"%(arm,ok,len(calls),len(sel),len(sel)-len(calls),ret))
