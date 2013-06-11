import subprocess
import time
import string
from datetime import datetime

#MAC={}

MAC=["0","0","0","0"]


current_router = 0

#



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
    elif(user_input=="q"):
      clean_up()
      break

def set_router_configuration():

  MAC_temp = MAC
  IP_address=""
  SSID="RX-99999005"
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

  for i in range (0,len(MAC_temp)):
    if(MAC_temp[i] == MAC_list[0]):
      index = i
      break


  if(index < 0):
    print "Error: MAC",MAC_list[0],"not in list"
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


def test_repeaters():
  print "\r\nEnter SSID: "
  #SSID=raw_input()\
  SSID="RX-99999005"
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
    print "\r\n\r\nPress [ENTER] on both netbooks at the same time"
    raw_input()
    
    #wait for other side to change APs
    time.sleep(3)
    for j in range(0,len(MAC_list)):
      if(i!=j):
        process = subprocess.Popen(["./wireless_ping.sh "+SSID+" "+MAC_list[j]+" "+source_IP+" "+destination_IP+" 10"], stdout=subprocess.PIPE,shell=True)
        test_results=process.communicate()[0]
        #print test_results
        if(string.find(test_results,"bytes from") == -1):
          print "Connection failed:",MAC_list[j],"->",MAC_list[i]
        else:
          print "Connection succeeded:",MAC_list[j],"->",MAC_list[i]
      

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
