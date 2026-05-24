#!/bin/sh

if [ ! -f ./gradlew ]; then
  echo "Error: Gradle wrapper not found. Please run this script from the root of the project."
  exit 1
elif [ ! -x ./gradlew ]; then
  echo "Error: Gradle wrapper is not executable. Please make it executable with 'chmod +x ./gradlew'."
  exit 1
fi

exit 0
