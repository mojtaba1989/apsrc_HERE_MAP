import xml.etree.ElementTree as ET
import pandas  as pd
import numpy as np
import os

# Parse the XML file
XML_NAME = '/home/moji/Downloads/routes_2024-07-15T10_04_35-04_00.xml'
tree = ET.parse(XML_NAME)
root = tree.getroot()

# Function to extract route data and return as a string
def get_route_data(route, i):
    route_data = []
    
    route_type = route.find('type').text
    route_data.append(f"Route Type: {route_type}")
    
    departure = route.find('departure')
    departure_time = departure.find('time').text
    departure_lat = departure.find('lat').text
    departure_lng = departure.find('lng').text
    departure_elv = departure.find('elv').text
    route_data.append(f"Departure Time: {departure_time}")
    route_data.append(f"Departure Location: Lat={departure_lat}, Lng={departure_lng}, Elv={departure_elv}")
    
    arrival = route.find('arrival')
    arrival_time = arrival.find('time').text
    arrival_lat = arrival.find('lat').text
    arrival_lng = arrival.find('lng').text
    arrival_elv = arrival.find('elv').text
    route_data.append(f"Arrival Time: {arrival_time}")
    route_data.append(f"Arrival Location: Lat={arrival_lat}, Lng={arrival_lng}, Elv={arrival_elv}")
    
    summary = route.find('summary')
    duration = summary.find('duration').text
    distance = summary.find('distance').text
    base_duration = summary.find('baseDuration').text
    route_data.append(f"Duration: {duration}")
    route_data.append(f"Distance: {distance}")
    route_data.append(f"Base Duration: {base_duration}")
    txt_data =  '\n'.join(route_data)

    FILE_NAME = os.path.basename(XML_NAME) + '_route_' + str(i) + '.txt'
    with open(FILE_NAME, 'w') as file:
        file.write(txt_data + '\n\n')

    waypoint_list = []
    waypoints = route.find('waypoints')
    for waypoint in waypoints.findall('waypoint'):
        waypoint_list.append(float(waypoint.text))
    waypoint_np = np.array(waypoint_list).reshape((-1,3))
    filler = np.zeros_like(waypoint_np)
    waypoint_np = np.hstack((waypoint_np, filler))
    headers = ['lat', 'lng', 'elv', 'speed', 'base_speed', 'action']
    df = pd.DataFrame(waypoint_np, columns=headers)
    
    actions = route.find('actions')
    for action in actions.findall('action'):
        offset = int(action.find('offset').text)
        df.loc[offset, "action"] = 1
    
    df.loc[:, "speed"] = -1
    df.loc[:, "base_speed"] = -1
    spans = route.find('spans')
    for span in spans.findall('span'):
        offset = int(span.find('offset').text)
        df.loc[offset, "speed"] = float(span.find('trafficSpeed').text)
        df.loc[offset, "base_speed"] = float(span.find('baseSpeed').text)
        traffic_speed = span.find('trafficSpeed').text
    
    for h in range(df.shape[0]):
        if (df.loc[h, "speed"]==-1):
            try:
                df.loc[h, "speed"]=df.loc[h-1, "speed"]
                df.loc[h, "base_speed"]=df.loc[h-1, "base_speed"]
            except:
                pass
    FILE_NAME = os.path.basename(XML_NAME) + '_route_' + str(i) + '.csv'
    df.to_csv(FILE_NAME)

i = 1
for route in root.findall('route'):
    route_data = get_route_data(route, i)
    i += 1

print("Data has been written to routes.txt")
