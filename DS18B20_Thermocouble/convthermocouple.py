fi=open("Typ-K.csv","r")
lastv=0
v=0
lastt=0
t=0
rs=""
wc=0
for l in fi.readlines():
	#print(l)
	l=l.replace(",",".")
	ll=l.split(";")
	i=0
	for n in ll[1:10]:
		s=ll[0][:-1]+""+str(i)
		lastv=v
		lastt=t
		v=float(n)
		t=float(s)
		if (int(v)!=int(lastv)):
			#print v,lastv
			#print t,lastt
			if (lastt<0):
				interp=lastt+(t-lastt)/(v-lastv)*(int(v-1)-lastv)
				#print (t-lastt)/(v-lastv)*(int(v-1)-lastv)
				print int(v-1),interp
			else:
				interp=lastt+(t-lastt)/(v-lastv)*(int(v)-lastv)
				#print (t-lastt)/(v-lastv)*(int(v)-lastv)
				print int(v),interp
			rs=rs+", %f" %(interp)
			wc=wc+1
		if (v<=-6):
			interp=lastt+(t-lastt)/(v-lastv)*(int(v)-lastv)
			print int(v),interp
			rs="{"
			wc=0
		if (v==0):
			rs=rs+", %f" %(0)
			wc=wc+1;
		i=i+1
print rs+"};"
print wc