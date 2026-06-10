python3 -c "import socket; s=socket.socket();   
s.bind(('0.0.0.0', 5761)); s.listen(1); print('Czekam...');   
conn,addr=s.accept(); print('Połączono:', addr)"  
