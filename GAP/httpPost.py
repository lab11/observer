import sys
import requests
import time
import math
import json

url = 'http://post.gatd.io/b1598a80-6edf-4a43-a395-57fe4ec8fc62'

while 1:
    data = sys.stdin.readline()
    # do stuff with data
    parsed_data = data.split(' ')
  
    parsed_data[len(parsed_data) - 1] = parsed_data[len(parsed_data) - 1].rstrip('\n')
 
    payload = {'id' : int(parsed_data[0]), 'time' : math.floor(time.time()*1000), 'temp' : float(parsed_data[1]), 'rh' : float(parsed_data[2]), 'press' : float(parsed_data[3]), 'light' : float(parsed_data[4]), 'pir': int(parsed_data[5]) }
    try:    
        if (int(parsed_data[5]) == 0 or int(parsed_data[5]) == 1):
            r = requests.post(url, data=json.dumps(payload))
    except requests.exceptions.ConnectionError:
         time.sleep(5)
    print  parsed_data[0]
    print  parsed_data[1]
    print  parsed_data[2]
    print  parsed_data[3]
    print  parsed_data[4]
    print  parsed_data[5]
    print  math.floor(time.time()*1000)
   
        
