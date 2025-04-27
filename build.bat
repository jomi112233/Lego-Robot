@echo off
REM Set up Java environment
set JAVA_HOME=C:\Progra~1\Java\jdk1.7.0_80
set PATH=%JAVA_HOME%\bin;%PATH%

REM Navigate to project root (optional if already there)
cd /d "%~dp0"

REM Compile the Java file
javac -encoding UTF-8 -d bin -cp lib/ev3classes.jar src/LineFollower.java

REM Run using Ant
ant run
