#!/usr/bin/env python
# common PID generator  Rob Chapman  Jan 18, 2011

import os, sys, time, traceback
import inspect
import os.path

# make script runable from anywhere from any os
abspath = os.path.abspath(__file__)
dirname = os.path.dirname(abspath)
os.chdir(dirname)

printme = 0

# read in a text file and generate a C header file and a Python file so a
# single list can be used to keep embedded and host software in sync

pydest = '../TimbreTalk/' # place a copy here for Timbre Talk
hdest = '../'

pidlist = []
pidused = [0]*256

''' Text file source looks like:
	ACK_BIT		0x80	used for tagging acked frames. High bit is set for acked frames. next high bit is sfs 1 or 2
	SPS_BIT		0x40	used for indicating sfs type if an acked frame
	PID_BITS	0x2F	used to mask off upper bits

	SPS			0x00	used for initializing SPS frame acks
	SPS_ACK		++		confirm sps
	
	macro_name  (some value)  some comments
'''
def readPids(file):
	if printme: print "Executing: ", inspect.stack()[0][3]
	global pidlist
	pidlast = 0
	lines = open(file, 'r').readlines()
	for line in lines:
		if line.strip():
			l = line.split(None, 1) # only split off first entry
			if l[0] != '//' and len(l) > 1: # remove comments
				if l[1][0] == '(':
					rest = l[1].rsplit(')',1)
					pid = rest[0] + ')'
					comment = rest[1].strip()
				else:
					rest = l[1].split(None, 1)
					pid = rest[0]
					if len(rest) > 1:
						comment = rest[1].strip()
					else:
						comment = ''
					if pid == '++': # increment from last id
						pidlast += 1
						pid = hex(pidlast)
					else:
						pidlast = int(pid, 0)
					if pidused[pidlast] == 0:
						pidused[pidlast] = l[0]
					else:
						raise Exception("PID %X declared by %s is already declared by %s." % (pidlast, l[0], pidused[pidlast]))
				pidlist.append([l[0],pid,comment])


''' C header looks like:
#ifndef PID_H
#define PID_H

#define ACK_BIT 0x80 	// used for tagging acked frames. High bit is set for acked frames. next high bit is sfs 1 or 2
#define SPS_BIT 0x40 	// used for indicating sfs type if an acked frame
#define PID_BITS 0x2F 	// used to mask off upper bits
...

// provide a macro of all pids - should only be for pids > WHO_PIDS and < MAX_PIDS
#define FOR_EACH_PID(P) \
	P(PID) \
#endif
'''
def generateC(file):
	if printme: print "Executing: ", inspect.stack()[0][3]
	file = open(file, 'w')
	file.write('// PID declarations  %s'%genby())
	file.write('#ifndef PID_H\n#define PID_H\n\n')
	for pid in pidlist:
		file.write('#define %s %s \t// %s\n'%(pid[0], pid[1], pid[2]))
	file.write('\n#define FOR_EACH_PID(P) \\\n')
	unique = {}
	for p in pidlist:
		try:
			int(p[1], 16)
			unique[p[1]] = p[0] # get unique list using hex value as key
		except:
			pass
	for pid in unique: file.write('\tP(%s) /* %s */ \\\n'%(unique[pid], pid))
	file.write('\n#endif')
	file.close()


''' Python looks like:
# packet types
ACK_BIT=0x80	# used for tagging acked frames. High bit is set for acked frames. next high bit is sfs 1 or 2
SPS_BIT=0x40	# used for indicating sfs type if an acked frame
PID_BITS=0x2F	# used to mask off upper bits

pids = {
	0x80:"ACK_BIT",	# used for tagging acked frames. High bit is set for acked frames. next high bit is sfs 1 or 2
	0x40:"SPS_BIT",	# used for indicating sfs type if an acked frame
	0x2F:"PID_BITS",	# used to mask off upper bits
}
'''
def generatePython(filename):
	if printme: print "Executing: ", inspect.stack()[0][3]
	import shutil
	file = open(filename, 'w')
	file.write('# PID declarations  %s'%genby())
	file.write('# packet types')
	for pid in pidlist:
		file.write('\n%s=%s\t# %s'%(pid[0], pid[1], pid[2]))
	file.write('\n\npids = {')
	for pid in pidlist:
		if pid is pidlist[-1]:
			comma = ''
		else:
			comma = ','
		file.write('\n\t%s:"%s"%s\t# %s'%(pid[0], pid[0], comma, pid[2]))
	file.write('\n}')
	file.close()

def genby(): # string for heading
	if printme: print "Executing: ", inspect.stack()[0][3]
	import time, os
	t = time.strftime('%b %d, %Y  %H:%M:%S',time.localtime())
	return 'generated by parsepids.py  %s\n\n'%t

def fileModTime(file): # return file modified date
	if printme: print "Executing: ", inspect.stack()[0][3]
	return time.localtime(os.path.getmtime(file))

def generatePids():
	readPids('pids.txt')
	generateC(hdest+'pids.h')
	if pydest:
		generatePython(pydest+'pids.py')

if __name__ == '__main__':
	try:
		hfile = hdest+'pids.h'
		if os.path.isfile(hfile):
			if fileModTime('pids.txt') > fileModTime(hfile) or \
				fileModTime('parsepids.py') > fileModTime('pids.txt'):
				generatePids()
		else:
			generatePids()
	except Exception as message:
		print type(message)     # the exception instance
		traceback.print_exc(file=sys.stderr)
