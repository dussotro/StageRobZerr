while 1:
        cmd=raw_input(">>>")
	fileout=open("cmd", "w")
	fileout.write(cmd)
	fileout.close()
