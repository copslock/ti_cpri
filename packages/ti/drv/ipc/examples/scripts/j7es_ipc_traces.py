#Python script to save the trace output to file
import vlab
import os
import csv
import string
import re
from collections import defaultdict

def main():    
    columns = defaultdict(list) 
    with open('outbuf.txt') as f:
        reader = csv.DictReader(f) 
        for row in reader: 
            for (k,v) in row.items(): 
                columns[k].append(v) 
         
                      
    #Open the trace file now...
    fd = open("ipc_test_trace.txt","w")

    for indx, adr in enumerate(columns['addr'],start=0):
        s1 = columns['space'][indx]
        
        #print(int(adr, 16), 1024, s1);
        arr = vlab.read_memory(int(adr, 16), 1024, s1) 
        s = str(arr)#, encoding='utf-8', errors='strict')

        #printable = set(string.printable)
        #filter(lambda x: x in printable, s)

        s = re.sub(r'[^\x00-\x7f]',r'', s) 
        print(s)
        #s = s.encode('ascii',errors='ignore').decode()
        #print(s)
        fd.write(s);

    # Close the file now..
    fd.close() 
    f.close()

if __name__ == '__main__':
  main()

