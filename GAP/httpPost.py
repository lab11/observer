import sys
import requests
import time
import math
import json
import logging
import logging.handlers

url = 'http://post.gatd.io/b1598a80-6edf-4a43-a395-57fe4ec8fc62'
url2 = 'http://post.gatd.io/ad06a282-03a5-4a67-baf1-b6e03ca70fd1'
url3 = 'http://post.gatd.io/687935b2-81c1-44ee-8f14-3195a9a95fb1'

log = logging.getLogger('wearabouts_log')
log.setLevel(logging.DEBUG)
log_filename = 'observer_log.out'
handler = logging.handlers.TimedRotatingFileHandler(log_filename, when='midnight', backupCount=7)
log.addHandler(handler)
log.info("Running wearabouts controller...")


while 1:
    data = sys.stdin.readline()
    # do stuff with data
    parsed_data = data.split(' ')
  
    parsed_data[len(parsed_data) - 1] = parsed_data[len(parsed_data) - 1].rstrip('\n')
 
    try:
	payload = {'id' : int(float(parsed_data[0])), 'temp' : float(parsed_data[1]), 'rh' : float(parsed_data[2]), 'press' : float(parsed_data[3]), 'light' : float(parsed_data[4]), 'pir': int(parsed_data[5]) }    

    	try:    
       	    if (int(parsed_data[5]) == 0 or int(parsed_data[5]) == 1):
                #r = requests.post(url, data=json.dumps(payload))
	        r2 = requests.post(url2, data=json.dumps(payload))
	        #r3 = requests.post(url3, data=json.dumps(payload))
        except requests.exceptions.ConnectionError:
            time.sleep(5)
    except ValueError:
	# ignore packet, it's gone to shit
	log.error("ValueError: parsed data: " + str(parsed_data) + "time: " + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) )

 
    print  parsed_data[0]
    print  parsed_data[1]
    print  parsed_data[2]
    print  parsed_data[3]
    print  parsed_data[4]
    print  parsed_data[5]
    print  math.floor(time.time()*1000)
   
        
