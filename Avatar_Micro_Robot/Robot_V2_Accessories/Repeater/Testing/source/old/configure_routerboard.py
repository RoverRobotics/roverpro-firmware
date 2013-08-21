import subprocess
import time
import string
from datetime import datetime

#MAC={}

MAC=["0","0","0","0"]


current_router = 0

#

#for local MAC function
SSID_number=""
channel_number=0

output_file_name = datetime.now().strftime("logs/log_%Y_%m_%d_%H_%M_%S.txt")

#output_file_name='test.txt'

f = open(output_file_name,'w')

time_start = time.time()

def main():


  initial_setup()

  while(True):
    print"\r\n\r\n\r\n"
    print "[a]uto find MAC addresses"
    print "[m]anually enter MAC addresses"
    print "[s]et router configuration"
    print "[c]heck MAC addresses"
    print "[d]isplay configuration"
    print "[t]est all repeaters"
    print "set router configuration with [l]ocal MAC"
    print "[r]eset router to default"
    print "[q]uit"

    user_input=raw_input("")
    if(user_input=="a"):
      auto_find_MAC()
    elif(user_input=="m"):
      manually_enter_MAC()
    elif(user_input=="s"):
      set_router_configuration()
    elif(user_input=="r"):
      reset_router_configuration()
    elif(user_input=="c"):
      check_MAC_addresses()
    elif(user_input=="d"):
      display_configuration()
    elif(user_input=="t"):
      test_repeaters()
    elif(user_input=="l"):
      local_MAC()
    elif(user_input=="q"):
      clean_up()
      break

def local_MAC():

  global SSID_number
  global channel_number
  use_stored_values = 0
  stored_values_valid = 0

  if(len(SSID_number) == 8):
    if(int(channel_number) >= 1) & (int(channel_number) <= 14):
      stored_values_valid=1


  while True:
    if(stored_values_valid):
        print "\r\n\r\nEnter 8 digit SSID number or 'r' to recall last values"
    else:
      print "\r\n\r\nEnter 8 digit SSID number"
    user_input=raw_input()
    if(len(user_input)==8):
      break
    else:
      if(user_input=='r'):
        print "Press [ENTER] to use these stored values:"
        print "SSID: RXR-"+str(SSID_number)
        print "Channel: "+str(channel_number)
        use_stored_values=1
        break
      print "Incorrect length.  Try again"

  if(use_stored_values==0):
    SSID_number = user_input

  SSID_number_int = int(float(SSID_number))
  SSID_number_hex = hex(SSID_number_int)
  
  #Generate SSID
  SSID = "RXR-"+SSID_number

  while(True):
    if(stored_values_valid & use_stored_values):
      break
    print "\r\n\r\nEnter channel number"
    user_input=raw_input()
    if((int(user_input) >= 1) & (int(user_input) <= 14)):
      break
    print "Enter a number between 1 and 14"
  
  if(use_stored_values==0):
    channel_number = user_input

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
  
  
  #MAC addresses that start with x2, x6, xA or xE are locally administered
  MAC_range = ""
  for i in range (0,4):

    MAC_range = hex(SSID_number_int%256).lstrip("0x")+MAC_range

    if((SSID_number_int%256) < 0x10):
      MAC_range = ":0"+MAC_range
    else:
      MAC_range = ":"+MAC_range

    SSID_number_int=int(SSID_number_int/256)
  MAC_range = "02"+MAC_range



  while(True):
    print "\r\n\r\nEnter repeater number (0 to exit):"
    user_input=raw_input()
    repeater_number=int(user_input)

    if(repeater_number==0):
      break;

    print SSID
    print repeater_number
    print MAC_range
    num_repeaters = 20

    argument_string =  MAC_range+" "+str(repeater_number)+" 10.1.123 "+SSID+" "+frequency+" "+str(num_repeaters)

    process = subprocess.Popen(["./custom_mac.sh "+argument_string], stdout=subprocess.PIPE,shell=True)
    process_output=process.communicate()[0]
    print process_output
  

def set_router_configuration():

  #use list() so both variables don't reference the same list
  MAC_temp = list(MAC)
  IP_address=""
  SSID="RX-99999002"
  frequency=2422


  file_write_line("Set router configuration")

  #MAC_list = return_MAC_list("MikroTik")

  #file_write_list("Current MACs on: ",MAC_list)

  MAC_list = return_MAC_list("MikroTik")
  while(len(MAC_list) > 1):
    print "Power down all repeaters except one wired to netbook, and hit Enter."
    raw_input()
    MAC_list = return_MAC_list("MikroTik")

#  print "Select MAC address of router:"
#  for i in range(0,len(MAC)):
#    print "[",i,"]: ",MAC[i]
#  user_input=raw_input()

#  index = int(float(user_input))
  
  index = -1
  if(len(MAC_list) > 0):
    for i in range (0,len(MAC_temp)):
      if(MAC_temp[i] == MAC_list[0]):
        index = i
        break
    if(index < 0):
      print "Error: MAC",MAC_list[0],"not in list"
      return
  else:
    print "Make sure router is reset to default first"
    return




  MAC_temp[index] = "0"
#  IP_address = "10.1.123."+str(index+3)
  IP_address="10.1.123.250"
  

#  if(user_input=="0"):
#    MAC_temp[0]="0"
#    IP_address="10.1.123.3"
#  elif(user_input=="1"):
#    MAC_temp[1]="0"
#    IP_address="10.1.123.4"
#  elif(user_input=="2"):
#    MAC_temp[2]="0"
#    IP_address="10.1.123.5"
#  elif(user_input=="3"):
#    MAC_temp[3]="0"
#    IP_address="10.1.123.6"

  file_write_line("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
  file_write_line("@MAC of router: "+MAC_list[0])
  file_write_list("@Mesh MAC list: ",MAC_temp) 
  file_write_line("@SSID: "+SSID)
  file_write_line("@Frequency: "+str(frequency))
  file_write_line("@IP address: "+IP_address)
  file_write_line("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

  #subprocess.call(['./python_helper.sh',MAC_temp[0],MAC_temp[1],MAC_temp[2],MAC_temp[3],IP_address,SSID,str(frequency)])

  argument_string =  MAC_temp[0]+" "+MAC_temp[1]+" "+MAC_temp[2]+" "+MAC_temp[3]+" "+IP_address+" "+SSID+" "+str(frequency)

  process = subprocess.Popen(["./python_helper.sh "+argument_string], stdout=subprocess.PIPE,shell=True)
  process_output=process.communicate()[0]
  print process_output
  file_write_line(process_output)
  f.write("------------------------------------\r\n\r\n\r\n")


def display_configuration():
  file_write_line("Display configuration")
  process = subprocess.Popen(["./display_configuration.sh"], stdout=subprocess.PIPE,shell=True)  
  process_output=process.communicate()[0]
  print process_output
  file_write_line(process_output)
  f.write("------------------------------------\r\n\r\n\r\n")

def manually_enter_MAC2():
  i=0
  while(True):
    print "Enter MAC (0 when done)"
    user_input=raw_input()
    if(user_input=="0"):
      break
    MAC[i]=user_input
    i=i+1

  print "Input: ",MAC

def manually_enter_MAC():

  file_write_line("Manually enter MAC")

  print "Enter MAC addresses (separated by spaces):"
  user_input=raw_input()
  MAC_list = user_input.split()
  
  for i in range (0,len(MAC_list)):
    if(i >= len(MAC)):
      break
    MAC[i] = MAC_list[i]
  print "Entered: ",MAC


  file_write_list("Entered MACS: ",MAC)

    
def check_MAC_addresses():

  file_write_line("Check MAC addresses")

  for i in range(0,len(MAC)):
    if(MAC[i] != "0"):
      process = subprocess.Popen(["sudo iwlist wlan0 scan | grep '"+MAC[i]+"'"], stdout=subprocess.PIPE,shell=True)
      MAC_string=process.communicate()[0]
      #print MAC_string
      if( (len(MAC_string) == 0) | (len(MAC_string) > 100)):
        output_string = "Error: MAC address "+MAC[i]+" not found"
      else:
        output_string = "MAC address "+MAC[i]+" found!"
      print output_string
      file_write_line(output_string)
    #MAC_list_raw=MAC_string.split()

def reset_router_configuration():
  subprocess.call(['sudo bash reset_routerboard.sh'],shell=True)

#returns a list of MAC addresses for routers with the given SSID
def return_MAC_list(SSID):
  process = subprocess.Popen(["sudo iwlist wlan0 scan | grep '"+SSID+"' -B5 | grep 'Address'"], stdout=subprocess.PIPE,shell=True)
  MAC_string=process.communicate()[0]
  MAC_list_raw=MAC_string.split()
  MAC_list=[]
  for i in range(0,len(MAC_list_raw)):
    if(((i+1)%5)==0):
      MAC_list.append(MAC_list_raw[i])
  #print MAC_list
  MAC_list.sort()
  return MAC_list



def auto_find_MAC():
  global MAC

  file_write_line("Auto find MAC")

  #process = subprocess.Popen(["sudo iwlist wlan0 scan | grep 'MikroTik' -B5 | grep 'Address' | cut -b 30-43"], stdout=subprocess.PIPE,shell=True)
  MAC_list = return_MAC_list("MikroTik")

  print "MAC addresses: ",MAC_list

  for i in range (0,len(MAC)):
    if(i < len(MAC_list)):
      MAC[i] = MAC_list[i]
    else:
      MAC[i] = "0"

  file_write_list("Found MAC addresses: ",MAC)

  print "MAC addresses: ",MAC

  MAC_string=""
  for i in range(0,len(MAC_list)):
    MAC_string=MAC_string+" "+MAC_list[i]

  print "Copy the following line to the clipboard in case the program crashes:"
  print MAC_string

  file_write_line("Better formatted MAC list: "+MAC_string)


def return_SSIDs():
  process = subprocess.Popen(["sudo iwlist wlan0 scan | grep 'ESSID' | grep 'RXR-'"], stdout=subprocess.PIPE,shell=True)
  SSID_string=process.communicate()[0]
  SSID_list_raw=SSID_string.split()
  SSID_list=[]
  for el in SSID_list_raw:
    new_SSID = el[7:len(el)-1]
    unique_SSID = 1
    for el2 in SSID_list:
      if(new_SSID == el2):
        unique_SSID = 0
    if(unique_SSID):
      SSID_list.append(el[7:len(el)-1])
    
  return SSID_list

def test_repeaters():
  print "Select SSID: \r\n"
  SSIDs =  return_SSIDs()
  for i,el in enumerate(SSIDs):
    print i,": ",el
  SSID_index=int(raw_input())
  #print "\r\nEnter SSID: "
  #SSID=raw_input()\
  #SSID="RX-99999005"
  SSID = SSIDs[SSID_index]
  MAC_list = return_MAC_list(SSID)
  print "\r\n",len(MAC_list),"repeaters found"

  print "\r\n\r\n[1]st Side"
  print "[2]nd Side"
  print "[s]low 2nd Side"
  Number=raw_input()
 

  if(Number=="1"):
    ping_test_side_1(SSID,MAC_list)
  elif(Number=="2"):
    ping_test_side_2(SSID,MAC_list,15) 
  elif(Number=="s"): 
    ping_test_side_2(SSID,MAC_list,45)
  
  #process = subprocess.Popen(["./check_mesh.sh "+SSID], stdout=subprocess.PIPE,shell=True)
  #test_results=process.communicate()[0]
  #print test_results

def ping_test_side_2(SSID,MAC_list,seconds):

  for i in range(0,len(MAC_list)):
    print "\r\n\r\nPress [ENTER] on both netbooks at the same time"
    raw_input()
    #source_IP = "10.1.123."+str(i+2)
    source_IP = "10.1.123.4"
    destination_IP = "10.1.123.3"
    print "Connecting to repeater",MAC_list[i]
    test_1_start_time = time.time()
    while( (time.time() - test_1_start_time) < seconds):
      process = subprocess.Popen(["./connect_to_AP.sh "+SSID+" "+MAC_list[i]+" "+source_IP], stdout=subprocess.PIPE,shell=True)
      test_results=process.communicate()[0]
      #print test_results

def ping_test_side_1(SSID,MAC_list):
  source_IP = "10.1.123.3"
  destination_IP = "10.1.123.4"



  for i in range(0,len(MAC_list)):
    error_output_string = "No errors"

    print "\r\n\r\nPress [ENTER] on both netbooks at the same time"
    user_input = raw_input()

    if(user_input == "d"):
      print error_output_string
    
    #wait for other side to change APs
    time.sleep(3)

    for j in range(0,len(MAC_list)):
      if(i!=j):
        process = subprocess.Popen(["./wireless_ping.sh "+SSID+" "+MAC_list[j]+" "+source_IP+" "+destination_IP+" 10"], stdout=subprocess.PIPE,shell=True)
        test_results=process.communicate()[0]
        #print test_results
        if(string.find(test_results,"bytes from") == -1):
          print "Connection failed:",MAC_list[j],"->",MAC_list[i]
          error_output_string=error_output_string+test_results
        else:
          print "Connection succeeded:",MAC_list[j],"->",MAC_list[i]

  print "\r\n\r\nPress [ENTER]"
  user_input = raw_input()

  if(user_input == "d"):
    print error_output_string
      

def initial_setup():
  #turning off network
  process = subprocess.Popen(["sudo stop network-manager"], stdout=subprocess.PIPE,shell=True)
  #subprocess.call(['sudo stop network-manager'],shell=True)
  

  time.sleep(1)
  is_wlan0_up=""

  #make sure wlan0 is brought up before continuing
  while(is_wlan0_up==""):
    process = subprocess.Popen(["sudo ifconfig | grep 'wlan0'"], stdout=subprocess.PIPE,shell=True)
    is_wlan0_up = process.communicate()[0]
    process = subprocess.Popen(["sudo ifconfig wlan0 up"], stdout=subprocess.PIPE,shell=True)


  #making sure eth0 is up
  process = subprocess.Popen(["sudo ifconfig eth0 up"], stdout=subprocess.PIPE,shell=True)



def clean_up():
  #start network manager again
  process = subprocess.Popen(["sudo start network-manager"], stdout=subprocess.PIPE,shell=True)

def file_timestamp():
  f.write(str(int(time.time()-time_start))+": ")

def file_write_line(theline):
  file_timestamp()
  f.write(theline+"\n")

def file_write_list(description,list_elements):
  file_timestamp()
  f.write(description)
  for el in list_elements:
    f.write(el+" ")
  f.write("\n")


#auto_find_MAC()
main()
