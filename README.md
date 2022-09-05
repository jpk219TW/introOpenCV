
# twinny_kt_nargo_leader_api
## Introduction
This project is a simple practice project for learning the basics of opencv and git

### API Document
http://docs.ros.org/melodic/api/rospy/html/rospy-module.html


### Last updated (2020.02.05)
New features are added and can be accessed via HTTP API. (copied from twinny template)

## Installation Guide

### Prerequisite
```
ros melodic
```
### Installation
First of all, download this repository.
```
git clone git@github.com:jpk219TW/introOpenCV.git
cd introOpenCV
...
catkin_make
```
### (Optional) Termination
To harvest all the threads made in the run-time session, send stop signal to the main process by hitting Ctrl+C. Some of child threads are killed by this command, but the main process will be still running. Then, pause the main process by hitting Ctrl+Z and then kill the main process by ```kill -9``` command with process ID to make sure the process is killed clearly.

## Demo / Example / Use-case / Run
Run this server with the following command
```
roslaunch introOpenCV total.launch
```

## References
* [Requirements documentation]()
* [Class specifications]()

## Contact
* @jpkk219TW jpk219@twinny.co.kr
