import os
import sys


#config 
parse_names =   [["sim_ticks","Number of ticks"],
                ["system.mem_ctrls.bw_write::total","Num of bw write back"]
               ]
vertical = True

# arguments
# 1. path to file from current directory
# 2. name of output file

def statParser(argv):
    if len(argv) != 3:
        print("Incorrect number of arguments")
        return False    
    
    path = os.path.join(os.getcwd(), argv[1])
    if os.path.isfile(path) == False:
        print("File does not exist")
        return False
    
    if argv[1].find("stats.txt") == -1:
        print("Point to stats.txt")
        return False
    
    with open(path, "r") as f:
        stats_text = f.read()
    
    csv_elems = []
    
    for stat_name, desc_name in parse_names :
        temp_begin = stats_text.find(stat_name,0)
        temp_begin = stats_text.find(r' ', temp_begin)
        
        while stats_text[temp_begin] == r' ':
            temp_begin = temp_begin + 1
        temp_end = stats_text.find(r' ',temp_begin)
        
        value = stats_text[temp_begin:temp_end]
        
        csv_elems.append([stat_name, desc_name, value])
            
    csv_text = "";
    if vertical:
        for elem in csv_elems:
            csv_text = csv_text + elem[1] + "," +  elem[2] + '\n' 
    else: 
        for i in range(2):
            for elem in csv_elems:
                csv_text = csv_text + elem[i+1] + ","
            csv_text = csv_text[0:len(csv_text)-1]
            csv_text = csv_text + "\n"
        
    with open(argv[2],'w+') as f :
        f.write(csv_text)
        
    print(csv_text)
    return True
    
if __name__ == "__main__":
    ret = statParser(sys.argv)
    if ret:
        print("Stats successfully parsed")
    else:
        print("Failed parse")