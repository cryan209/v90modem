import socket, sys, time
port=int(sys.argv[1]); out=sys.argv[2]
data=(b"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 0123456789\r\n")*80
s=socket.create_connection(('127.0.0.1',port)); s.sendall(data)
open(out+'.sent','wb').write(data)
got=b''; s.settimeout(1.0); t0=time.time()
while time.time()-t0<60:
    try:
        d=s.recv(4096)
        if not d: break
        got+=d
    except socket.timeout: pass
open(out,'wb').write(got); print('received',len(got))
