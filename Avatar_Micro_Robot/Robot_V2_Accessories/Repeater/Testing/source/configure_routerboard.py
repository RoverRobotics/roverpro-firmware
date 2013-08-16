import subprocess
import time
import sys

computer_IP = "5.5.5.100"
wired_repeater_IP="192.168.88.1"
new_repeater_IP = "5.5.5.1"

def call_linux_command(command):
  process = subprocess.Popen([command], stdout=subprocess.PIPE,shell=True)
  #process = subprocess.call([command], stdout=subprocess.PIPE,shell=True)
  process_output=process.communicate()[0]
  #process.terminate()
  return process_output
  
def main_menu():

  initial_setup()
  
  while(True):
    print "Select option:"
    print "[0] Initial Builder Setup"
    print "[1] Initial Tester Setup"
    print "[2] Reconfigure"
    print "[3] Display settings"
    print "[q] Quit"
    
    user_input=raw_input("")
    
    if(user_input=="0"):
      initial_builder_setup()
    elif(user_input=="1"):
      initial_tester_setup()
    elif(user_input=="2"):
      reconfigure()
    elif(user_input=="3"):
      display_settings()
    elif(user_input=="q"):
      clean_up()
      break
      

def return_MAC_list():
  MAC_string = call_linux_command("sudo iwlist wlan0 scan | grep '02:0' ")
  MAC_list = []
  MAC_list_raw = MAC_string.split()
  for i,el in enumerate(MAC_list_raw):
    if((i+1)%5==0):    
      MAC_list.append(el)
  return MAC_list

def return_MAC(network_number,repeater_number):
  #MAC addresses that start with x2, x6, xA or xE are locally administered
  MAC_range = ""
  network_number_int = int(float(network_number))
  repeater_number_int = int(float(repeater_number))
  for i in range (0,4):

    #new_byte = network_number_int%256
    #new_byte_padded = 
    MAC_range = hex(network_number_int%256).lstrip("0x")+MAC_range

    if((network_number_int%256) == 0):
      MAC_range = ":00"+MAC_range
    elif((network_number_int%256) < 0x10):
      MAC_range = ":0"+MAC_range
    else:
      MAC_range = ":"+MAC_range

    network_number_int=int(network_number_int/256)
  MAC_range = "02"+MAC_range
  if(repeater_number_int == 0):
    MAC = MAC_range+":00"
  elif(repeater_number_int < 0x10):
    MAC = MAC_range+":0"+hex(repeater_number_int).lstrip("0x")
  else:
    MAC = MAC_range+":"+hex(repeater_number_int).lstrip("0x")
  return MAC
  
def return_network_number(MAC):
  MAC_hex = ""
  split_MAC =  MAC.split(":") 
  for i in range(1,5):
    MAC_hex=MAC_hex+split_MAC[i]
  MAC_int = str(int(MAC_hex,16)).zfill(8)
  return MAC_int
  

def return_WLAN_frequency(channel_number):
  if(channel_number=="1"):
    frequency="2412"
  elif(channel_number=="2"):
    frequency="2417"
  elif(channel_number=="3"):
    frequency = "2422"
  elif(channel_number=="4"):
    frequency = "2427"
  elif(channel_number=="5"):
    frequency = "2432"
  elif(channel_number=="6"):
    frequency = "2437"
  elif(channel_number=="7"):
    frequency = "2442"
  elif(channel_number=="8"):
    frequency = "2447"
  elif(channel_number=="9"):
    frequency = "2452"
  elif(channel_number=="10"):
    frequency = "2457"
  elif(channel_number=="11"):
    frequency = "2462"
  elif(channel_number=="12"):
    frequency = "2467"
  elif(channel_number=="13"):
    frequency = "2472"
  elif(channel_number=="14"):
    frequency = "2484"
  else:
    frequency = "0"

  return frequency
      
 
def ping_wait(IP):

  time.sleep(5)
  print "Waiting for reboot..."
  command_string = "ping -W 1 -c 1 "+IP+" | grep 'bytes from'"
  while(True):
    if(call_linux_command(command_string) == ""):
      pass
    else:
      print "Reboot finished!"
      time.sleep(2)
      print ""
      return

def reset_configuration():
  print "Resetting configuration..."
  wired_setup()
  print call_linux_command("sudo bash reset_configuration.sh")
  
def wired_setup():
  print call_linux_command("sudo bash wired_setup.sh")
  
def wireless_setup(SSID,MAC):
  #print "Connecting to"+SSID+" ("+MAC+")"
  call_linux_command("sudo bash wireless_setup.sh '"+SSID+"' '"+MAC+"'")

def wireless_config(old_SSID, new_SSID, old_MAC, new_MAC, repeater_number, frequency,old_repeater_IP,new_repeater_IP):
  wireless_setup(old_SSID, old_MAC)
  time.sleep(2)
  check_wireless_connection(old_SSID,old_MAC)
  general_config(new_SSID,new_MAC, repeater_number, frequency,old_repeater_IP,new_repeater_IP)

def check_wireless_connection(SSID, MAC):
  output = call_linux_command("sudo iwconfig wlan0 | grep 'Not-Associated'")
  if(output == "" ):
    print "Wireless connection successful!"
    return 1
  else:
    print "Wireless connection to "+SSID+" ("+MAC+") failed!"
    return 0

  
def send_DSA_key():
  print call_linux_command("sudo bash send_DSA_key.sh")

def general_config(new_SSID, new_MAC, repeater_number, frequency, old_IP, new_IP):
  print "Configuring repeater"
  return call_linux_command("sudo bash configure.sh '"+new_SSID+"' '"+new_MAC+"' '"+repeater_number+"' '"+frequency+"' '"+old_IP+"' '"+new_IP+"'")
      
def initial_builder_setup():
  print "Make sure that Ethernet cable is wired to repeater and repeater is powered up"

  new_SSID="RXR-00000000"
  repeater_number="1"
  new_MAC=return_MAC(new_SSID.lstrip("RXR-"),repeater_number)

  old_repeater_IP = "192.168.88.1"

  frequency = return_WLAN_frequency("3")
 
  wired_setup()
  reset_configuration()
  ping_wait(wired_repeater_IP)
  send_DSA_key()
  print general_config(new_SSID, new_MAC, repeater_number,frequency,old_repeater_IP,new_repeater_IP)
  ping_wait(wired_repeater_IP)
  
  old_SSID=new_SSID
  old_MAC=new_MAC

  new_SSID="RXR-00000001"
  new_MAC=return_MAC(new_SSID.lstrip("RXR-"),repeater_number)
  old_repeater_IP = "5.5.5.1"

  wireless_config(old_SSID, new_SSID, old_MAC, new_MAC, repeater_number, frequency,old_repeater_IP,new_repeater_IP)

  #general_config(new_SSID,new_MAC, repeater_number, frequency,old_repeater_IP,new_repeater_IP)

def connect_to_repeater_from_list():
  old_repeater_IP="5.5.5.1"

  wireless_setup("none","00:00:00:00:00:00")
  time.sleep(2)

  MAC_list = return_MAC_list()
  
  Network_list = []
  print "Choose repeater:"
  for i, el in enumerate(MAC_list):
    Network_list.append(return_network_number(el))
    print "["+str(i)+"] "+Network_list[i]+" ("+el+")"
  repeater_index=int(raw_input())

  network_number=Network_list[repeater_index]
  SSID="RXR-"+network_number
  MAC=MAC_list[repeater_index]

  wireless_setup(SSID,MAC)

  return SSID,MAC

def display_settings():  
  connect_to_repeater_from_list()
  print call_linux_command("sudo bash display_configuration.sh '5.5.5.1'")
  
def reconfigure():

  (old_SSID,old_MAC) = connect_to_repeater_from_list()

  print "Enter new network number: "
  new_network_number=raw_input()
  print "Enter new repeater number: "
  new_repeater_number = raw_input()
  print "Enter channel: "
  channel = raw_input()

  new_MAC = return_MAC(new_network_number,new_repeater_number)
  new_SSID = "RXR-"+new_network_number
  frequency = return_WLAN_frequency(channel)




  wireless_config(old_SSID, new_SSID, old_MAC, new_MAC, new_repeater_number, frequency,old_repeater_IP,new_repeater_IP)
  
  
def initial_setup():
  call_linux_command("sudo stop network-manager")
  
def clean_up():
  call_linux_command("sudo start network-manager")
  
main_menu()
  
  
