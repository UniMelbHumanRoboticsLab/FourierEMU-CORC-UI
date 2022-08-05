# FLNL server (see https://github.com/vcrocher/libFLNL)
#
# Vincent Crocher - Unimelb - 2022
# 
# Apache 2.0 License

import socket
import sys
import threading
import struct
import time


def Checksum(b):
	ck = 0
	for i in range(2, len(b)-1):
		ck = ck ^ b[i]
	return ck


class FLNLServer:

	def __init__(self):
		self.newCmdRcv = False
		self.newValsRcv = False
		self.CmdRcv = ""
		self.Connected = False
		self.receiving=False
	
	#NOT TESTED
	def ProcessRcvValues(self, data, nbvals):
		for i in range(nbvals):
			self.ValsRcv[i] = struct.unpack('d', data[i*8])[0]
	
	def recFct(self):
		while self.receiving:
			data = self.connection.recv(255)
			if data:
				if data[0]==ord('C'):
					#print('Cmd: '+str(data[2:5]))
					self.newCmdRcv = True
					self.CmdRcv = data[2:5].decode("utf-8")
					nbvals = data[1]
					self.ProcessRcvValues(data[6:6+8*nbvals], nbvals)
				if data[0]==ord('V'):
					nbvals = data[1]
					#print('Values ('+str(nbvals)+')')
					self.ProcessRcvValues(self, data[2:2+8*nbvals], nbvals)
					self.newValsRcv = True
	
	def WaitForClient(self, ip="127.0.0.1", port=2042):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server_address = (ip, port)
		self.sock.bind(server_address)
		self.sock.listen(1)
		
		# Wait for a connection
		print('FLNL: Waiting for a connection ('+ip+':'+str(port)+')...')
		self.connection, client_address = self.sock.accept()
		print('FLNL: Connected!');
		self.Connected = True

		#Create reception thread
		self.receiving=True
		recPr = threading.Thread(target=self.recFct, daemon=True)
		recPr.start()

	def IsConnected(self):
		return self.Connected

	def SendValues(self, vals):
		if(self.Connected):
			if(len(vals)>31):
				print('FLNL: Error, too many values to send')
				return
		
			#Build packet header to send:
			tosend=bytearray(255);
			tosend[0]=ord('V');
			tosend[1]=len(vals);
			
			#Pack double values
			i=2;
			for val in vals:
				val_b=bytearray(struct.pack("d", val))
				for byte in val_b:
					tosend[i]=byte
					i=i+1
			
			tosend[255-1]=Checksum(tosend);
			
			#send
			self.connection.sendall(tosend)
		


	def SendCmd(self, cmd, vals=None):
		if(self.Connected):
			if vals is None: 
				vals = []
		
			#Build packet header to send:
			tosend=bytearray(255);
			tosend[0]=ord('C');
			tosend[1]=len(vals);
			
			#Command
			cmd_bytes=cmd.encode()
			i=2
			for byte in cmd_bytes:
				tosend[i]=byte
				i=i+1
			for k in range(4-len(cmd_bytes)):
				tosend[i]=0
				i=i+1
			
			#Pack double values
			for val in vals:
				val_b=bytearray(struct.pack("d", val)) 
				for byte in val_b:
					tosend[i]=byte
					i=i+1
			
			tosend[255-1]=Checksum(tosend);
			
			#send
			self.connection.sendall(tosend)


	def IsCmd(self, cmd):
		if self.newCmdRcv and self.CmdRcv==cmd:
			print(self.CmdRcv)
			self.newCmdRcv=False;
			return True
		else:
			return False


	def GetCmd(self):
		if(self.newCmdRcv):
			self.newCmdRcv=False;
			return CmdRcv
		else:
			return ""


	def Close(self):
		self.Connected=False
		self.receiving=False;
		self.connection.close()