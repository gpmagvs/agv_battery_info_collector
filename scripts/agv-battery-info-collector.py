#!/usr/bin/env python3
import rospy
import http.client
import socket
import json
import argparse
import threading
from gpm_msgs.msg import ModuleInformation,BatteryState

latest_battery_data = None
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


def PostBatInfoToServer(batinfo, fieldName, agvName, host):
    
    headers = {'Content-Type': 'application/json'}
    try:
        status, response_data = post_request("http://{}".format(host), "/api/AGVBattery?fieldName={}&agvName={}".format(fieldName, agvName), headers, batinfo)
        #print(response_data)
        
        return status == 200  # Return True if the request was successful (HTTP 200), otherwise False
	
    except Exception as e:
        rospy.logerr("HTTP request failed: {}".format(e))
        return False

def ModuleInformationCallback(moduleinfo):
    data = {
        "batteryID": moduleinfo.Battery.batteryID,
        "state": moduleinfo.Battery.state,
        "errorCode": moduleinfo.Battery.errorCode,
        "Voltage": moduleinfo.Battery.Voltage,
        "batteryLevel": moduleinfo.Battery.batteryLevel,
        "chargeCurrent": moduleinfo.Battery.chargeCurrent,
        "dischargeCurrent": moduleinfo.Battery.dischargeCurrent,
        "maxCellTemperature": moduleinfo.Battery.maxCellTemperature,
        "minCellTemperature": moduleinfo.Battery.minCellTemperature,
        "cycle": moduleinfo.Battery.cycle,
        "chargeTime": moduleinfo.Battery.chargeTime,
        "useTime": moduleinfo.Battery.useTime,
        "tagNumber": moduleinfo.reader.tagID
    }
    global latest_battery_data
    latest_battery_data = data

def check_host_connection(host):
    try:
        socket.gethostbyname(host)
        return True
    except socket.error:
        return False
def battery_info_collector():
    rospy.init_node('battery_info_collector', anonymous=True)
    host = rospy.get_param('~host', 'localhost')
    agvName = rospy.get_param('~agvName', 'AGV1')
    fieldName = rospy.get_param('~fieldName', 'Unknown_Region')
    pubInterval = rospy.get_param('~pubInterval', 10)
    rospy.loginfo("Starting battery_info_collector with host: {}, agvName: {}, fieldName: {}, pubInterval: {}".format(host, agvName, fieldName, pubInterval))
    

    rospy.Subscriber('/module_information', ModuleInformation, ModuleInformationCallback)
    rospy.loginfo("Battery info collector node started.")

    if not check_host_connection(host):
        rospy.logwarn("Failed to connect to host: {}. Test communication status failed.".format(host))

    def publish_battery_data(fieldName, agvName,host,pubInterval):
        global latest_battery_data
        while not rospy.is_shutdown():
                if latest_battery_data:
                    rospy.logwarn("PostBatInfoToServer ...")
                    success = PostBatInfoToServer(latest_battery_data,fieldName,agvName,host)
                    if not success:
                        rospy.logwarn("PostBatInfoToServer failed...")
                    rospy.sleep(pubInterval)
                else:
                    rospy.logwarn("No battery data received yet...")
                    rospy.sleep(3)

    # Start the publishing thread
    thread = threading.Thread(target=publish_battery_data, args=(fieldName, agvName,host,pubInterval,))
    thread.daemon = True  # This ensures the thread will exit when the main program does
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:        
        battery_info_collector()
    except rospy.ROSInterruptException:
        pass
