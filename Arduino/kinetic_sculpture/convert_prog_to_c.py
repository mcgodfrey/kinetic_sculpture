
with open("program.txt",'r') as f:
	num_servos=0
	for line in f:
		if line[0]=='#':
			continue
		for ind,item in enumerate(line.split(','))
			t,pos=item.split('=')
			print "prog_code.all_servos[{}].times[{}]={};".format(num_servos,ind,t)
			print "prog_code.all_servos[{}].positions[{}]={};".format(num_servos,ind,pos)
		print "prog_code.all_servos[{}].n={};".format(num_servos,ind)
		num_servos++

	print "#define NUM_SERVOS {}".format(num_servos)

