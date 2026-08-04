import neurapy
from neurapy.robot import Robot
import sys
import prettytable
import win32api

# Import der NeuraPy Version & Klasse
from neurapy.robot import VERSION

print(f"Python Version    : {sys.version.split()[0]}")
print(f"Prettytable       : {prettytable.__version__}")
print(f"NeuraPy SDK       : {VERSION}")
#Robot() und der print kann nur dann erfolgreich ausgefuehrt werden, wenn der Roboter über Ethernet verbuden ist, die 
r = Robot()
print(f"Robot Name        : {r.robot_name}")

'''
OUTPUT sollte in etwa so aussehen:
#WICHTIG: robot_name muss "lara5" sein, sonst ist die Verbindung nicht korrekt aufgebaut!
Python Version    : 3.14.0
Prettytable       : 3.18.0
NeuraPy SDK       : v5.0.8
Robot Name        : lara5
'''