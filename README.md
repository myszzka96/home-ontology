# home-ontology
An application that helps you remember where you last put an item in your home. 

On the Python side, we are using Google Speech-To-Text and Text-To-Speech API services to get 
user questions and respond with answers. We are also using RASA NLU to parse the intention of 
a user question and generate a set of keywords. These keywords are then sent to the Java side.

On the Java side, we receive the keywords and use them to build a SPARQL query. We then use 
the OWLAPI v5.1.12 to query Description Logic (DL) triples from an ontology describing some 
common household items and their locations. 

## Java Requirements
Note: the following paths are for Ubuntu 16.04, other OS distributions may download Java to 
a different path.
- Apache Maven 3.6.3
	- Maven home: /usr/local/apache-maven/apache-maven-3.6.3
	- Java version: 1.8.0_242, vendor: Private Build, runtime: /usr/lib/jvm/java-8-openjdk-amd64/jre
	- Default locale: en_US, platform encoding: UTF-8
	- OS name: "linux", version: "4.15.0-96-generic", arch: "amd64", family: "unix"
- Java 8
	- \openjdk version "1.8.0_242"
	- OpenJDK Runtime Environment (build 1.8.0_242-8u242-b08-0ubuntu3~16.04-b08)
	- OpenJDK 64-Bit Server VM (build 25.242-b08, mixed mode)

## Python requirements
- To be listed ...

### Add this line to your .bashrc
```
export GOOGLE_APPLICATION_CREDENTIALS='<path_to_google_cloud_texttospeech_credentials.json>'
```

## Environmental Variables
Add the following to your ~/.bashrc in Linux systems. For Windows, you would use "Set Environmental Path"
but I don't know where the specific path Java Development Kit (JDK) and Maven are installed.
```bash
# Java paths
export J2SDKDIR=/usr/lib/jvm/java-8-openjdk-amd64
export J2REDIR=/usr/lib/jvm/java-8-openjdk-amd64
export PATH=$PATH:/usr/lib/jvm/java-8-openjdk-amd64/bin:/usr/lib/jvm/java-8-openjdk-amd64/db/bin
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export DERBY_HOME=/usr/lib/jvm/java-8-openjdk-amd64/db

# Maven paths
export MAVEN_HOME=/usr/local/apache-maven/apache-maven-3.6.3
export M2_HOME=/usr/local/apache-maven/apache-maven-3.6.3
export M2=$M2_HOME/bin
export PATH=$M2:$PATH
```

## Run Java Application
The name of the target and the version can be found in the pom.xml file.
```bash.sh
cd home_ontology_app_java
chmod +x start.sh
./start.sh build-run-log
```
The result of the SPARQL query will be stored in home_ontology_app_java/results/result.txt.
The contents of this file will be used to generate the response that the Python application
will speak. Currently, if you display the contents of the file:
```bash
less results/result.txt  
```
Your terminal will display a single triple:
```
orange isLocatedInside Fridge
```
For help on different commands you can run on the application, type:
```bash
./start.sh
```

## Train dataset with Rasa NLU
This will allow for script to train the robot on the model. You do not need to redo it as it's been already done but if you want to train other models go for it.
```
python -m rasa_nlu.train -c config/object_config.yml --data data/training_dataset_object.json -o output --fixed_model_name RasaOnto --project RasaOnto --verbose
```

## Run slang onto program
To run the script you need to do the following:
```
source ~/catkin_ws/devel/setup.bash
```
If running on the robot got to ```hsr_b mode```. On a separate terminal run ros master ```roscore```. Then in the orginal window run:
```
rosrun home-ontology test_weather.py
```
