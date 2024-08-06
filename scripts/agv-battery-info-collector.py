#!/usr/bin/env python3
import rospy
import http.client
import json
import argparse
from gpm_msgs.msg import ModuleInformation,BatteryState

def post_request(url, endpoint, headers, data):

    parsed_url = url.replace('http://', '').replace('https://', '')
    host = parsed_url.split('/')[0]

    connection = http.client.HTTPConnection(host)
    
    json_data = json.dumps(data)
    connection.request("POST",  endpoint, body=json_data, headers=headers)
    
    response = connection.getresponse()
    
    response_data = response.read().decode()
    
    connection.close()
    
	
    return response.status, response_data


def PostBatInfoToServer(batinfo, fieldName, agvName,host):
    
    headers = {'Content-Type': 'application/json'}
    try:
        status, response_data = post_request("http://{}".format(host),"/api/AGVBattery?fieldName={}&agvName={}".format(fieldName,agvName),headers,batinfo)
        #print(response_data)
	
    except Exception as e:
        rospy.logerr("HTTP request failed: {}".format(e))
        return False

def ModuleInformationCallback(moduleinfo, args):
    fieldName, agvName,host = args
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
        "tagNumber":moduleinfo.reader.tagID
        }
    PostBatInfoToServer(data, fieldName, agvName,host)


def battery_info_collector(fieldName, agvName,host):
    rospy.init_node('battery_info_collector', anonymous=True)
    #rospy.Subscriber('battery_level', Float32, battery_callback)
    rospy.Subscriber('/module_information', ModuleInformation, ModuleInformationCallback, callback_args=(fieldName, agvName,host))
    rospy.loginfo("Battery info collector node started.")
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Battery info collector for AGV.')
    parser.add_argument('--fieldName', type=str, required=True, help='Field name for the API request')
    parser.add_argument('--agvName', type=str, required=True, help='AGV name for the API request')
    parser.add_argument('--host', type=str, required=True, help='HOST URL for the API request')
    args = parser.parse_args()
    rospy.loginfo(args)
    try:
        battery_info_collector(args.fieldName,args.agvName,args.host)
    except rospy.ROSInterruptException:
        pass
