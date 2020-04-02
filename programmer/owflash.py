#!/usr/bin/python
# Copyright (c) 2015, Tobias Mueller tm(at)tm3d.de
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are 
# met: 
# 
#  * Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer. 
#  * Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the 
#    distribution. 
#  * All advertising materials mentioning features or use of this 
#    software must display the following acknowledgement: This product 
#    includes software developed by tm3d.de and its contributors. 
#  * Neither the name of tm3d.de nor the names of its contributors may 
#    be used to endorse or promote products derived from this software 
#    without specific prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 




import time
import subprocess
import sys

def owcom(dev,send,rc):
	res=[]
	f=open("/sys/bus/w1/devices/%s/rw" %(dev),"r+b",0)
	f.write("".join(map(chr, send)))
	if (rc!=0):
		res=map(ord,f.read(rc))
	f.close()
	return res
		

def crc8(arr):
	lscrc=0x0;
	for v in arr:
		bit=1;
		while bit<256:
			if (v&bit)==bit:
				lb=1
			else:
				lb=0
			if (lscrc&1)!=lb:
				lscrc=(lscrc>>1)^0x8c 
			else:
				lscrc=(lscrc>>1)
			bit=bit*2
	return lscrc
	
def testnr(s):
	for c in s.lower()[:]:
		if not((c) in ['0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','-']):
			return False
	return True
 
def addid(id,val):
	for i in range(7):
		id[i+1]=id[i+1]+val
		if id[i+1]>254:
			id[i+1]=id[i+1]-254
			val=1
		else:
			return id
	return id
#

print "Open Hex File ...",
fi=open(sys.argv[1],"r")
data=[]
for l in fi.readlines():
	sys.stdout.write(".")
	sys.stdout.flush()
	bc=int(l[1:3],16)
	fw=int(l[3:7],16)
	ty=int(l[7:9],16)
	chsm=bc+(fw>>8)+(fw&0xFF)+ty
	for i in range(bc):
		p=9+(i*2)
		d=int(l[p:p+2],16)
		chsm=(chsm+d)&0xFF
		data.append(d)
	chsm=(chsm+int(l[9+bc*2:11+bc*2],16))&0xFF
	if (chsm!=0):
		print "Error Checksum...."
		exit()
	#print bc,fw,ty,chsm
fi.close()
print

l=subprocess.check_output("ls /sys/bus/w1/devices/", shell=True)
dc=0
dl=[]
for g in  (l.split("\n")):
	if len(g)>2:
		if testnr(g[0:2]):
			dl.append(g)
			dc=dc+1
			print dc,") ",g
if dc==0:
	print "No 1-Wire Device found"
	exit(0)
n=int(raw_input("No. of Device: "))
n=n-1
if (n>dc-1)or(n<0):
		exit(0)
s=dl[n]
sys.stdout.write('Go to Flashmode....')
sys.stdout.flush()
owcom(s,[0x88],0)
owcom(s,[0x88],0)
owcom(s,[0x88],0)
for i in range (20):
	l=subprocess.check_output("ls /sys/bus/w1/devices/", shell=True)
	dc=0
	dl=[]
	sys.stdout.write(".")
	sys.stdout.flush()
	for g in  (l.split("\n")):
		if (g[0:15]=="a3-55aa55aa55aa"):
			break
	if (g[0:15]!="a3-55aa55aa55aa"):
		time.sleep(1)
if (g[0:15]=="a3-55aa55aa55aa"):
	print "found"
else:
	print "ERROR Enter Flashmode!" 
	exit()
f=open("/sys/bus/w1/devices/w1_bus_master1/w1_master_remove","r+b",0)
f.write(s)
f.close()
time.sleep(5)
s="a3-55aa55aa55aa"
prog=data
l=len(prog)
if (l>7616):
	print "Code to big  ... Max 7616 Byte (119 Pages)"
	exit()
pages= l/64
for i in range(64-(l%64)):
	#print i
	prog.append(0xFF)
pages= len(prog)/64
if (pages>119):
	print "Code to big  ... Max 7616 Byte (119 Pages)"
	exit()


print "Programm Page (of ", pages,")"
	
for i in range(pages):
	sys.stdout.write("%i " % (i+1) )
	sys.stdout.flush()

	h=i*64;
	hl=h&0xFF
	hh=h>>8
	#print hh, hl
	mem=[hl,hh]+prog[h:h+64]
	erroc=0
	while (1):
		owcom(s,[0x0F]+mem,0) 
		rmem=owcom(s,[0xAA],66)
		if (rmem!=mem):
			print rmem
			erroc=erroc+1
			if erroc>5:
				print "WRITING ERROR ... "
				exit()
			continue
		owcom(s,[0x55],0)	
		time.sleep(0.05)
		owcom(s,[0xB8,hl,hh],0)
		time.sleep(0.05)
		rmem=owcom(s,[0xAA],66)
		if (rmem!=mem):
			print "error in flash"
			print mem
			print rmem
			erroc=erroc+1
			if erroc>5:
				print "WRITING ERROR ... "
				exit()
			continue
		#for v in rmem:
		#	print "%02X " % (v),
		break
print "\nReset AVR"
owcom(s,[0x89],0)
time.sleep(1)
f=open("/sys/bus/w1/devices/w1_bus_master1/w1_master_remove","r+b",0)
f.write("a3-55aa55aa55aa")
f.close()	
	
	




#mem=[0x00,0x2]
#for i in range (64):
#	mem.append(i)
#owcom(s,[0x0F]+mem,0)
#rmem=owcom(s,[0xAA],70)
#print rmem
#owcom(s,[0x55],0)
#time.sleep(0.05)
#owcom(s,[0xB8,0x00,0x02],0)
#time.sleep(0.05)
#rmem=owcom(s,[0xAA],70)
#print rmem
#for v in rmem:
#	print "%02X " % (v)
		

		
