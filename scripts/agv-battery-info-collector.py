#!/usr/bin/env python3
import rospy
import http.client
import socket
import json
import threading
from gpm_msgs.msg import ModuleInformation,BatteryState
from gpm_msgs.msg import DriversState

latest_battery_data = {
    "batteryID": 1,         
    "state": 0,             
    "errorCode": 0,         
    "Voltage": 2330,           
    "batteryLevel": 90,      
    "chargeCurrent": 0,     
    "dischargeCurrent": 1220,  
    "maxCellTemperature": 40,
    "minCellTemperature": 30,
    "cycle": 120,             
    "chargeTime": 0,        
    "useTime": 0,           
    "tagNumber": 0,
    "batCellsStates": []          
}

latest_cell_states = [
    {"name": "Cell1", "voltage": 1300, "errorCode": 1, "state": 0},
    {"name": "Cell2", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell3", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell4", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell5", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell6", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell7", "voltage": 2300, "errorCode": 1, "state": 0},
    {"name": "Cell8", "voltage": 3300, "errorCode": 1, "state": 0}
]
voltageRatio = 1;
currentRatio =1;
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
    status = 0
    rospy.loginfo("batinfo:{}".format(batinfo));
    try:
        status, response_data = post_request("http://{}".format(host), "/api/AGVBattery?fieldName={}&agvName={}".format(fieldName, agvName), headers, batinfo)
        print(response_data)
        if status != 200:
            rospy.logerr("PostBatInfoToServer(Host:{},FieldName:{},AgvName:{}) failed. Status Code :{}".format(host,fieldName,agvName,status))
            return False
        return True
	
    except Exception as e:
        rospy.logerr("HTTP request failed,status: {},error: {}".format(status,e))
        return False
def BatteryCellStateCallback(driversState):
    global latest_cell_states
    # Extract voltage and errorCode from each driver state
    latest_cell_states = []
    for driver in driversState.driversState:
        cell_state = {
            "name": driver.name,
            "voltage": driver.voltage,
            "errorCode": driver.errorCode,
            "state": driver.state
        }
        latest_cell_states.append(cell_state)
        rospy.loginfo("Driver {}: voltage={}, errorCode={}".format(
            driver.name,
            driver.voltage,
            driver.errorCode
        ))
    rospy.loginfo("Latest cell states: {}".format(latest_cell_states))

def ModuleInformationCallback(moduleinfo):
    global voltageRatio
    global currentRatio
    global latest_cell_states
    data = {
        "batteryID": moduleinfo.Battery.batteryID,
        "state": moduleinfo.Battery.state,
        "errorCode": moduleinfo.Battery.errorCode,
        "Voltage": moduleinfo.Battery.Voltage * voltageRatio,
        "batteryLevel": moduleinfo.Battery.batteryLevel,
        "chargeCurrent": moduleinfo.Battery.chargeCurrent * currentRatio,
        "dischargeCurrent": moduleinfo.Battery.dischargeCurrent * currentRatio,
        "maxCellTemperature": moduleinfo.Battery.maxCellTemperature,
        "minCellTemperature": moduleinfo.Battery.minCellTemperature,
        "cycle": moduleinfo.Battery.cycle,
        "chargeTime": moduleinfo.Battery.chargeTime,
        "useTime": moduleinfo.Battery.useTime,
        "tagNumber": moduleinfo.reader.tagID,
        "batCellsStates": latest_cell_states
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
    global voltageRatio
    global currentRatio
    rospy.init_node('battery_info_collector', anonymous=True)
    host = rospy.get_param('~host', '192.168.206.1:5254')
    agvName = rospy.get_param('~agvName', 'OHAGV_001')
    fieldName = rospy.get_param('~fieldName', 'K7')
    pubInterval = rospy.get_param('~pubInterval', 5)
    voltageRatio = rospy.get_param('~voltageRatio', 1)
    currentRatio = rospy.get_param('~currentRatio', 1)
    rospy.loginfo("Starting battery_info_collector with host: {}, agvName: {}, fieldName: {}, pubInterval: {}, voltageRatio: {}, currentRatio: {}".format(host, agvName, fieldName, pubInterval, voltageRatio, currentRatio))
    

    rospy.Subscriber('/module_information', ModuleInformation, ModuleInformationCallback)
    rospy.Subscriber('/battery_cell_state', DriversState, BatteryCellStateCallback)
    rospy.loginfo("Battery info collector node started.")

    if not check_host_connection(host):
        rospy.logwarn("Failed to connect to host: {}. Test communication status failed.".format(host))

    def publish_battery_data(fieldName, agvName,host,pubInterval):
        global latest_battery_data,latest_cell_states
        while not rospy.is_shutdown():
                if latest_battery_data:
                    latest_battery_data["batCellsStates"] = latest_cell_states
                    rospy.logwarn("PostBatInfoToServer(Host:{},FieldName:{},AgvName:{}) ...".format(host,fieldName,agvName))
                    success = PostBatInfoToServer(latest_battery_data,fieldName,agvName,host)
                    if not success:
                        rospy.logwarn("PostBatInfoToServer(Host:{}) failed...".format(host))
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
