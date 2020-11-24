#! /usr/bin/env python

#intended for use if the robot is not avaiable. 
#Requires setting up a service key with Google Cloud
#Uses OWLAPI to connect to ontology
#Uses Rasa for the NLU
#Uses Google Cloud TTS & STT

import os #to call terminal commands
import rospy, hsrb_interface, rospkg, math, yaml, sys
import numpy as np
from std_msgs.msg import String

import pyaudio
import speech_recognition as sr

from google.cloud import texttospeech

from rasa_nlu.model import Interpreter

from rospkg import RosPack

#to play the converted audio
import os #you might need to get an audio player for terminal 'apt-get install mpg123'

# Initialization -----------------------------------------------------------------------------------------

#print("Taking control of the robot's interface")

#robot = hsrb_interface.robot.Robot()

#tts = robot.get('default_tts')
#tts.language = tts.ENGLISH

#print("Took control")

#make sure to use below line to add the path to your env to get the API key
# export GOOGLE_APPLICATION_CREDENTIALS="path_to_\slang\Palpi-project.json"

m = sr.Microphone()
r = sr.Recognizer()

rp = RosPack()

interpreter = Interpreter.load(rp.get_path('home-ontology') + '/output/RasaOnto/RasaOnto')

#Helper functions -----------------------------------------------------------------------------------------
# takes only the important data from the parsed object
def objectify(parsed):
    acc = {}
    acc['intent'] = parsed[u'intent'][u'name']
    for p in parsed[u'entities']:
        acc[p[u'entity']] = p[u'value']
    return acc 

def publish_weather(ws):
    # hsrb_mode triggers rosmaster node IP address = hsrb.local:11311 (have to be in RoboCanes-5G network)
    # sim_mode triggers rosmaster node IP address = your local IP address
    # Run roscore to init ros_master node, and in another terminal window run 'sim_mode' before this script.
    rospy.init_node('weather_publisher')
    pub = rospy.Publisher('weather', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown():
      weather_str = ws #% rospy.get_time()
      rospy.loginfo(weather_str)
      pub.publish(weather_str)
      rate.sleep() #it gives some time before speaking?

def speak_wait(s):
  nwords = len(s.split(" "))
  sperword = 0.4
  publish_weather(s)
  tts.say(s)
  rospy.sleep(nwords*sperword)

def speak_comp(s):
  nwords = len(s.split(" "))
  sperword = 0.4 
  # Instantiates a client
  client = texttospeech.TextToSpeechClient()
  # Set the text input to be synthesized
  synthesis_input = texttospeech.types.SynthesisInput(text=s)
  # Build the voice request, select the language code ("en-US") and the ssml with voice gender ("neutral")
  voice = texttospeech.types.VoiceSelectionParams(language_code='en-US', ssml_gender=texttospeech.enums.SsmlVoiceGender.NEUTRAL)
  # Select the type of audio file you want returned
  audio_config = texttospeech.types.AudioConfig(audio_encoding=texttospeech.enums.AudioEncoding.MP3)
  #Perform the text-to-speech request on the text input with the selected
  # voice parameters and audio file type
  publish_weather(s)
  response = client.synthesize_speech(synthesis_input, voice, audio_config)
  print('response.audio_content')
  print(type(response.audio_content))

  # The response's audio_content is binary.
  with open('output.mp3', 'wb') as out:
    # Write the response to the output file.
    out.write(response.audio_content)
    print('Audio content written to file "output.mp3"')
  os.system('mpg123 output.mp3')
  os.remove('output.mp3') #removes the mp3 file


def recognize(prompt, timeout = 5):
  while timeout > 0:
    timeout -= 1
    speak_comp(prompt)
    try:
      with m as source: 
        audio = r.listen(source)
      print('source')
      print(type(source))

      w = r.recognize_google(audio).lower()
      recognized = True
      return w
    except (sr.UnknownValueError):
      speak_comp("Sorry I couldn't hear that.")
    except (sr.RequestError):
      speak_comp("Google speech recognition failure.")
      fail()

def find_object(item, count): 
  speak_comp("Ok! Let's get to work.")
  #TODO: run and accept result of a terminal command 
  #This will be the OWLAPI in Java result
  if item == "bull":
    item = "bowl"
  item = item.capitalize()
  res = "/home/kasia/catkin_ws/src/home-ontology/home_ontology_app_java/results/result.txt"
  cat = "rm " + res
  os.system(cat)
  comm = "java -jar /home/kasia/catkin_ws/src/home-ontology/home_ontology_app_java/target/home_ontology_app-0.0.1.jar "+ item +" >> " + res
  os.system(comm)
  #rospy.sleep(5)
  #it needs to output a text file with an answer that then we open here

  #answer = open("/home/kasia/catkin_ws/src/home-ontology/QL_output/test.txt", "r")
  answer = open("/home/kasia/catkin_ws/src/home-ontology/home_ontology_app_java/results/result.txt", "r")
  print "this would be the file: ", answer.name #confirm by printing file name
  #here we need to read it and parse the answer
  #what is the answer format we are getting?
  if answer.mode == 'r':
    content = answer.readlines() #content to variable?
    if len(content) == 0:
      ans = "Sorry, I don't think I have seen " + item
      speak_comp(ans)
      answer.close()
      return -1

    for x in content:
      #here we need to read and parse depending on the format
      #we save the answer as a string

      #First, we need to deal with item not in the ontology

      #this is to get the object
      words = x.split()
      obj = words[0]

      #this is to get the home equipment
      equip = words[2]
      #print (" What we got: words[0], words[1], words[2]")

      #this is to get a string with nicely written position
      position = " is located somwhere around"
      if(words[1] == "isLocatedToRightOfHomeEquipment"):
        position = " to the right of"
      elif(words[1] == "isLocatedToLeftOfHomeEquipment"):
        position = " to the left of"
      elif(words[1] == "isLocatedBelowHomeEquipment"):
        position = " below"
      elif(words[1] == "isLocatedOnTopOfHomeEquipment"):
        position = " on top of"
      elif(words[1] == "isLocatedInsideHomeEquipment"):
        position = " inside"
      #here we skip to next line if we are getting parent isLocated or other thing
      else:
        continue 

      to_read = "The " + obj + " is located" + position + " " + equip + "."
      speak_comp(to_read)

  
  answer.close()
  

if __name__ == "__main__":

  tlk = "Adjusting for ambient noise, please be quiet"
  speak_comp(tlk)

  with m as source: r.adjust_for_ambient_noise(source)

  rospy.sleep(1)
  check = True
  count = 0
  while check:
    if count == 0:
      s = recognize("Hi, user! My name is Palpi! What would you like me to find?").lower()
    elif count == 1:
      s = recognize("Anything else I can look for?").lower()
    else:
      s = recognize("Can I find anything else for you?").lower()
    print("heard: " + s)
    print(type(s))
    parsed = interpreter.parse(s)
    obj = objectify(parsed)
    
    print(obj)

    intent = obj['intent']

    if intent == 'find':
      if 'object' in obj:
        item = obj['object']
        case = find_object(item, count) #if -1 we can do estimate where it is
        count=count+1
      else:
        speak_comp('Sorry, could you repeate that? My ears are a bit stuffed.')
        count=1

    elif intent == 'negative':
      speak_comp('Ok! Happy to be of assistance. Bye!')
      break
