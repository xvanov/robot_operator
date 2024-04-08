# Robot Operations 

This repo contains code pertaining to operating the robots on a high level. It can be
thought of as a wrapper that launches relevant processes, useful for non-coder operators
to launch, stop, debug and interact with wall panel team software. This includes
launching terminals, rviz, relevant operation commands and saving logs.

## Local Operation
Launches all necessary processes for a operator at a botbuilt facility. 
To install `Launcher` locally:
```
cd ./robot_operation/robot_launcher
./install.sh
```
To run launcher:
```
cd ./robot_operation/robot_launcher
./launch_robots.sh
```

## Remote Operation
### Setup on wall panel team side
Launch the server
```
ros2 run robot_operation launcher_server
```

### Setup on operator side
Launch the client (operator can interact with client in the same terminal)
```
ros2 run robot_operation launcher_client
```
Stream logs (operator can stream logs in a different terminal)
```
ros2 echo /robot_logs
```
