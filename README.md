import os
import sys
import RPi.GPIO as  GPIO
from time import sleep
import time
import urllib.request		
from time import sleep          
import serial               
import webbrowser
import cv2
import smtplib,ssl
from picamera import PiCamera
import speech_recognition as sr
r = sr.Recognizer()
speech = sr.Microphone(device_index=2)
camera = PiCamera()
def listen1():
    with sr.Microphone(device_index =1
                       ) as source:
               r.adjust_for_ambient_noise(source)
               print("Say Something");
               audio = r.listen(source)
               print("got it");
    return audio
def voice(audio1):
       try: 
         text1 = r.recognize_google(audio1) 
##         call('espeak '+text, shell=True) 
         print ("you said: " + text1);
         return text1; 
       except sr.UnknownValueError: 
          call(["espeak", "-s140  -ven+18 -z" , "Google Speech Recognition could not understand"])
          print("Google Speech Recognition could not understand") 
          return 0
       except sr.RequestError as e: 
          print("Could not request results from Google")
          return 0

        
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LCD_RS = 21
LCD_E  = 20
LCD_D4 = 16
LCD_D5 = 26
LCD_D6 = 19
LCD_D7 = 13

GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005


def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display




  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)



lcd_init()
lcd_byte(0x01,LCD_CMD)
lcd_string("   WELCOME",LCD_LINE_1)











def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    #print("NMEA Time: ", nmea_time,'\n')
    #print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    try:
        lat = float(nmea_latitude)                  #convert string into float for calculation
        longi = float(nmea_longitude)               #convertr string into float for calculation
    except:
        lat=0
        longi=0
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    

def send_sms():
    print("sending SMS..")

    cmd='AT\r\n'
    ser.write(cmd.encode())
    time.sleep(2)
    rcv = ser.read(20)
    print(rcv)
    cmd='AT+CMGF=1\r\n'
    ser.write(cmd.encode())
    time.sleep(2)
    rcv = ser.read(20)
    print(rcv)                                             
    phno="8179284344"                          
    cmd='AT+CMGS="'+str(phno)+'"\r\n'
    ser.write(cmd.encode())
    rcv = ser.read(20)
    print(rcv)                        
    time.sleep(1)
    cmd="women is in danger at"
    ser.write(cmd.encode())  # Message
    cmd=map_link
    ser.write(cmd.encode())  # Message
    #ser.write(msg.encode())  # Message
    time.sleep(1)
    cmd = "\x1A"
    ser.write(cmd.encode()) # Enable to send SMS
    time.sleep(10)
    print('SMS Sent')
    time.sleep(1)

def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position


gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyUSB0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0

ss=4
ss1=6
buz=17
GPIO.setup(ss, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(ss1, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(buz,GPIO.OUT)
GPIO.output(buz,False)
time.sleep(2)
kk=0
sp=0
hb=0


kk=0
try:
        while(True):
            while(1):
           
                audio1 = listen1() 
                text = voice(audio1);
                if 'help' in text:        
                    received_data = (str)(ser.readline())                   #read NMEA string received
                    GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string
                    if(kk==0):
                        lat_in_degrees=0
                        lat_in_degrees=0
                    if (GPGGA_data_available>0):
                        kk=1
                        GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
                        NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
                        GPS_Info()                                          #get time, latitude, longitude
                        map_link = 'http://maps.google.com/?q=' + str(lat_in_degrees) + ',' + str(long_in_degrees)    #create link to plot location on Google map
                    
                    map_link = 'http://maps.google.com/?q=' + str(lat_in_degrees) + ',' + str(long_in_degrees)    #create link to plot location on Google map
                    print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
                    print()


                    lcd_byte(0x01,LCD_CMD)
                    lcd_string("Sending info..",LCD_LINE_1)
                    print('Sending info...')
                    camera.capture("image.jpg")
                    cv2.waitKey(1)
                    send_sms()                

            

            sval=GPIO.input(ss)        
            print("B:" + str(sval))
            
           
 
            if(sval==0):
                lcd_byte(0x01,LCD_CMD)
                lcd_string("In danger",LCD_LINE_1)
                
                GPIO.output(buz,True)
                ii=0
                canc=0
                while(ii<20):
                    ii=ii+1
                    time.sleep(0.25)
                    if(GPIO.input(ss1)==0):
                        canc=1
                if(canc==0):
                    lcd_byte(0x01,LCD_CMD)
                    lcd_string("Sending info..",LCD_LINE_1)
                    print('Sending info...')
                    camera.capture("image.jpg")
                    cv2.waitKey(1)
                    bot.sendmessage()
                    send_sms()


except KeyboardInterrupt:
    webbrowser.open(map_link)        #open current position information in google map
    sys.exit(0)

