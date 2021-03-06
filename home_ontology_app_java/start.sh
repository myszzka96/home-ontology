#!/bin/bash


print_usage() {
  echo "=============================="
  echo "   Home Ontology App: Usage   "
  echo "=============================="
  echo "To build the application:"
  echo "  ./start.sh build"
  echo ""
  echo "To run the application:"
  echo "  ./start.sh run"
  echo ""
  echo "To build, then run the application, them open result log:"
  echo "  ./start.sh build-run-log"
  echo ""
  echo "To run the unit tests:"
  echo "  ./start.sh test"
  echo ""
}

# Name of file to write query result to.
result_dir='./results'
result_filename="result.txt"
result_filepath="$result_dir/$result_filename"

# Function to run the .jar application.
run_app() {
  printf "Welcome to the Home Ontology Application!\n\n"
  java -jar target/home_ontology_app-0.0.1.jar >> $result_file
}

# Function to build the .jar application, run it, make results folder,
# save results in a file inside results folder, and show contents of file.
run_app_extended() {
  printf "Welcome to the Home Ontology Application!\n\n"
  mkdir -p $result_dir

  if [ -f $result_filepath ]; then
    rm $result_filepath
  fi

  java -jar target/home_ontology_app-0.0.1.jar >> $result_filepath
  less $result_filepath
}

# If "rebuild", use Maven to clean and rebuild target .jar application.
# If "run", run the .jar application generated by the build.
if [ "$#" -eq 1 ]; then
  if [ $1 == "build" ]; then
    mvn clean
    mvn package
  elif [ $1 == "run" ]; then
    run_app
  elif [ $1 == "build-run-log" ]; then
    mvn clean
    mvn package
    run_app_extended
  elif [ $1 == "test" ]; then
    mvn test
  else
    print_usage
  fi
else
 print_usage
fi
