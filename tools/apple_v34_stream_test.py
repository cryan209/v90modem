#!/usr/bin/env python3
"""Hardware-free V.34 socket seam checks, including fragmented sample bytes."""
import os
from pathlib import Path
import socket
import subprocess
import threading

ROOT=Path(__file__).resolve().parents[1]


def run(direction, truncated=False):
    a,b=socket.socketpair();errors=[]
    def echo():
        try:
            if truncated:return
            while True:
                data=a.recv(511)
                if not data:break
                # Odd fragment boundaries exercise both C read-full loops;
                # the bytes themselves must remain exactly unchanged.
                for pos in range(0,len(data),73):a.sendall(data[pos:pos+73])
        except (BrokenPipeError,ConnectionResetError):pass
        except Exception as error:errors.append(error)
        finally:a.close()
    worker=threading.Thread(target=echo);worker.start()
    try:
        env=dict(os.environ,V34_DUPLEX_HARDWARE_FD=str(b.fileno()),V34_DUPLEX_HARDWARE_DIRECTION=direction)
        peer=subprocess.Popen([str(ROOT/'v34_duplex_test'),'2400','9600'],env=env,pass_fds=[b.fileno()],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        b.close()
        output,_=peer.communicate(timeout=30)
        assert peer.returncode==(1 if truncated else 0),output
        if truncated:assert "Hardware sample channel ended" in output,output
        if not truncated:assert 'trained=1/1' in output and 'errors=0/0' in output,output
        assert not errors,errors
    finally:
        b.close()
        if 'peer' in locals() and peer.poll() is None:peer.kill();peer.wait()
        a.close();worker.join(timeout=5)
    print(direction,'truncated channel rejected' if truncated else 'fragmented waveform exchange: training and error-free payload passed')


if __name__=='__main__':
    for fd in ['0','-2','not-a-descriptor']:
        check=subprocess.run([str(ROOT/'apple_usb_modem_coupler'),'--loopback-test','digital','--loopback-peer-fd',fd],capture_output=True,timeout=5)
        assert check.returncode==2,check
    run('caller');run('answer');run('caller',truncated=True)
