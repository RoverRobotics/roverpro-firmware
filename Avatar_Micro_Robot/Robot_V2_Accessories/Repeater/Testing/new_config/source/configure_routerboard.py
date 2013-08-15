import subprocess


def call_linux_command(command):
  process = subprocess.Popen([command], stdout=subprocess.PIPE,shell=True)
  process_output=process.communicate()[0]
  return process_output
  
def main_menu():

  initial_setup()
  
  while(True):
    print "Select option:"
    print "[0] Initial Builder Setup"
    print "[1] Initial Tester Setup"
    print "[2] Reconfigure"
    print "[q] Quit"
    
    user_input=raw_input("")
    
    if(user_input=="0"):
      initial_builder_setup()
    elif(user_input=="1"):
      initial_tester_setup()
    elif(user_input=="2"):
      reconfigure()
    elif(user_input=="q"):
      clean_up()
      break
      

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

  command_string = "ping -W 1 -c 1 "+IP+" | grep 'bytes from'"
  while(True):
    if(call_linux_command(command_string) == ""):
      print ".",
    else:
      print ""
      return

def reset_configuration():
  wired_setup()
  print call_linux_command("sudo bash reset_configuration.sh")
  
def wired_setup():
  print call_linux_command("sudo bash wired_setup.sh")
  
def wireless_setup(old_SSID,old_MAC):
  print call_linux_command("sudo bash wireless_setup.sh '"+old_SSID+"' '"+old_MAC+"'")
  
def send_DSA_key():
  print call_linux_command("sudo bash send_DSA_key.sh")

def general_config(new_SSID, new_MAC, frequency):
  print call_linux_command("sudo bash general_config.sh '"+new_SSID+"' '"+new_MAC+"' '"+frequency+"'")
      
def initial_builder_setup():
  print "Make sure that Ethernet cable is wired to repeater and repeater is powered up"
  wired_repeater_IP="192.168.88.1"
  old_SSID="NA"
  new_SSID="RXR-00000000"
  old_MAC="NA"
  repeater_number="1"
  new_MAC=return_MAC(new_SSID.lstrip("RXR-"),repeater_number)
  computer_IP = "5.5.5.100"
  old_repeater_IP = "5.5.5.1"
  new_repeater_IP = "5.5.5.1"
  frequency = return_WLAN_frequency("3")
  
  wired_setup()
  reset_configuration()
  ping_wait(wired_repeater_IP)
  send_DSA_key()
  general_config(new_SSID, new_MAC, frequency)
  ping_wait(wired_repeater_IP)
  wireless_setup()
  time.sleep(2)
  general_config(new_SSID,new_MAC, frequency)
  
  
  
  
def initial_setup():
  call_linux_command("sudo stop network-manager")
  
def clean_up():
  call_linux_command("sudo start network-manager")
  
main_menu()
  
  