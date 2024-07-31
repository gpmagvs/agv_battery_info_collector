#!/usr/bin/env python3
import rospy
#import httplib2
import http.client
import json
import argparse
import threading
import time
from gpm_msgs.msg import ModuleInformation,BatteryState

def post_request(url, endpoint, headers, data):

    parsed_url = url.replace('http://', '').replace('https://', '')
    host = parsed_url.split('/')[0]
    path = '/' + '/'.join(parsed_url.split('/')[1:])

    connection = http.client.HTTPConnection(host)
    
    json_data = json.dumps(data)
    connection.request("POST",  endpoint, body=json_data, headers=headers)
    
    response = connection.getresponse()
    
    print(response)
    response_data = response.read().decode()
    
    print(response_data)
    connection.close()
    
	
    return response.status, response_data

def PostBatInfoToServer(batinfo, fieldName, agvName):
    headers = {'Content-Type': 'application/json'}
    try:
        status, response_data = post_request("http://192.168.206.1:5254","/api/AGVBattery?fieldName={}&agvName={}".format(fieldName,agvName),headers,batinfo)
        print(response_data)
    except Exception as e:
        rospy.logerr("HTTP request failed: {}".format(e))
        return False

def ModuleInformationCallback(moduleinfo, args):
    fieldName, agvName = args
    data={
        "batteryID":moduleinfo.Battery.batteryID,
        "state":moduleinfo.Battery.state,
        "errorCode":moduleinfo.Battery.errorCode,
        "Voltage":moduleinfo.Battery.Voltage,
        "batteryLevel":moduleinfo.Battery.batteryLevel,
        "chargeCurrent":moduleinfo.Battery.chargeCurrent,
        "dischargeCurrent":moduleinfo.Battery.dischargeCurrent,
        "maxCellTemperature":moduleinfo.Battery.maxCellTemperature,
        "minCellTemperature":moduleinfo.Battery.minCellTemperature,
        "cycle":moduleinfo.Battery.cycle,
        "chargeTime":moduleinfo.Battery.chargeTime,
        "useTime":moduleinfo.Battery.useTime,
        }
    PostBatInfoToServer(data, fieldName, agvName)



def post_data_to_backend(fieldName, agvName,voltage):
    while not rospy.is_shutdown():
        _voltage=0
        if voltage is None:
            _voltage = 2310
        else:
            _voltage = voltage
        data={
        "batteryID":1,
        "state":1,
        "errorCode":0,
        "Voltage":_voltage,
        "batteryLevel":49,
        "chargeCurrent":0,
        "dischargeCurrent":23100,
        "maxCellTemperature":23,
        "minCellTemperature":12,
        "cycle":123,
        "chargeTime":2,
        "useTime":232,
        }  
        #print(data)
        PostBatInfoToServer(data, fieldName, agvName)
        time.sleep(0.1)

def battery_info_collector(fieldName, agvName,voltage):
    rospy.init_node('battery_info_collector_'+agvName, anonymous=True)
    #rospy.Subscriber('battery_level', Float32, battery_callback)
    #rospy.Subscriber('/module_information', ModuleInformation, ModuleInformationCallback, callback_args=(fieldName, agvName))
    rospy.loginfo("Battery info collector SIM started.")
    post_thread = threading.Thread(target=post_data_to_backend, args=(fieldName, agvName,voltage))
    post_thread.daemon = True
    post_thread.start()
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Battery info collector for AGV.')
    parser.add_argument('--fieldName', type=str, required=True, help='Field name for the API request')
    parser.add_argument('--agvName', type=str, required=True, help='AGV name for the API request')
    parser.add_argument('--voltage', type=int, required=False, help='AGV voltage simulation')
    args = parser.parse_args()
    rospy.loginfo(args)
    try:
        battery_info_collector(args.fieldName,args.agvName,args.voltage)
    except rospy.ROSInterruptException:
        pass
