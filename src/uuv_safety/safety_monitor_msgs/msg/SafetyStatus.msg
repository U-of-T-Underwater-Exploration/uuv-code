# Published by safety_monitor on /safety/status
uint8 OK = 0
uint8 WARN = 1
uint8 FAULT = 2

std_msgs/Header header
uint8 state
string[] reasons