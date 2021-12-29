main_path = "/home/sebastian/Documents/problemaEspecial/"

flight_data_filename = main_path + "flightDataV8.txt"
path_data_filename = main_path + "path_v8_pc55.txt"
errorLog_filename = main_path + "errorDataV8.txt"

file = open(path_data_filename)
lines = file.readlines()

# Utility function to convert string to list
def toList(string):
    l = []
    for x in string:
        l.append(x)
    return l
  
# Utility function to convert list to string
def toString(l):
    return ''.join(l)


