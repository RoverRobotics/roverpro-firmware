import subprocess
import time
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


  while(True):
    print"\r\n\r\n\r\n"
    print "[a]uto find MAC addresses"
    print "[m]anually enter MAC addresses"
    print "[s]et router configuration"
    print "[c]heck MAC addresses"
    print "[d]isplay configuration"
    print "[r]eset router to default"

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

def set_router_configuration():

  MAC_temp = MAC
  IP_address=""
  SSID="RX-99999005"
  frequency=2422


  file_write_line("Set router configuration")

  MAC_list = return_MAC_list("MikroTik")

  file_write_list("Current MACs on: ",MAC_list)

  

  print "Select MAC address of router:"
  for i in range(0,len(MAC)):
    print "[",i,"]: ",MAC[i]
  user_input=raw_input()

  index = int(float(user_input))

  MAC_temp[index] = "0"
  IP_address = "10.1.123."+str(index+3)
  

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
  file_write_line("@Option"+user_input+" selected: "+MAC[index])
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
  subprocess.call(['sudo sh reset_routerboard.sh'],shell=True)

#returns a list of MAC addresses for routers with the given SSID
def return_MAC_list(SSID):
  process = subprocess.Popen(["sudo iwlist wlan0 scan | grep '"+SSID+"' -B5 | grep 'Address'"], stdout=subprocess.PIPE,shell=True)
  MAC_string=process.communicate()[0]
  MAC_list_raw=MAC_string.split()
  MAC_list=[]
  for i in range(0,len(MAC_list_raw)):
    if(((i+1)%5)==0):
      MAC_list.append(MAC_list_raw[i])
  print MAC_list
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
