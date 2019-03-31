'''
The MIT License (MIT)
Copyright (c) 2013 Dave P.
'''
import json
import signal
import sys
from SimpleWebSocketServer import WebSocket, SimpleWebSocketServer, SimpleSSLWebSocketServer

import socket
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.Takeoff import Takeoff
from messages.Land import Land
#from messages.Move import Move
#from messages.Arm import Arm

#server_address = ("127.0.0.1", 10000)
#server_address = ("208.113.133.197", 25500)
server_address = ("192.168.100.17", 25500)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class SimpleEcho(WebSocket):

   def handleMessage(self):
      print 'I received ', self.data
      #header = ROSLinkHeader(1, 8, 100, 100, 1 )
      s.sendto(self.data, server_address)
      #self.sendMessage(json.dumps(header.__dict__).decode('utf8'))

      #self.sendMessage(self.data)

   def handleConnected(self):
      pass

   def handleClose(self):
      pass


if __name__ == "__main__":

   host = ''
   port = 8000 
   cls = SimpleEcho

   server = SimpleWebSocketServer(host, port, cls)

   def close_sig_handler(signal, frame):
      server.close()
      sys.exit()

   signal.signal(signal.SIGINT, close_sig_handler)

   server.serveforever()
