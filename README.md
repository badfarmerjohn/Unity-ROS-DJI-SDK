# Unity-ROS-DJI-SDK
A ROSBridgeLib client for communicating with DJI's ROS wrapper for their Onboard SDK.


## Software Installation
This repository is designed to be a submodule in an existing Unity project (it should live as a subfolder in  the Assets folder). It also depends on a ROSBridgeLib submodule (which is a Unity ROSBridge client implementation). From the root folder of the Unity project, run these commands:
```
git submodule add https://github.com/jeshlee121/ROSBridgeLib.git ./Assets/ROSBridgeLib/
git submodule add https://github.com/badfarmerjohn/Unity-ROS-DJI-SDK.git ./Assets/ROS-DJI-SDK/

git submodule init
git submodule update
```
If those commands fail (usually due to "authentication" issues), you will need to resolve those first. Try searching for a solution online.
*Note: the ROSBridgeLib submodule has another submodule (PointCloud) within it that may not be cloned recursively. In that case, you'll need to `cd` into the ROSBridgeLib folder and run `git submodule init` and `git submodule update` again.*

### Script Configuration
You need one DJI_SDK.cs script attached to one active scene object *per drone*. Each script represents one drone. You may have multiple of these in the scene, so long as they are attached to different objects.  
You will need to edit two properties/fields of each instance using the Unity scene editor (aka not through modifying the DJI_SDK.cs code) to specify which drone that instance will be:
  * `onboard_computer_address`: this is the network IP address of the drone's onboard computer. This is the computer running the ROSBridge server and ROS Onboard SDK node.
    * Ideally, the Unity computer should be on the same subnet as the drone's onboard computer or the drone's onboard computer should somehow be accessible to the Unity computer.
  * `rosbridge_port`: this is the port that the ROSBridge server on that drone's onboard computer is configured to use.
    * You can find this on the console output on the terminal that's running ROSBridge server.
  * `isM100`: this flag tells the script whether the drone being represented is a Matrice100. This is important as the API for Matrice 100s are a bit different than the rest of the DJI products.
With this configuration completed, the script will handle the rest (setup/teardown of the connection, requesting control authority).
  * TODO: may need to rework managing control authority to make it more robust.

### Simulation
Always test code with the drone in simulation mode first before running the code with the rotors actually engaged. You can enabe simulation by opening DJI Assistant 2 while the drone is on and connected to the computer. Go to the Simulation tab and open the simulator. Then **press Start Simulation**. Opening the simulator does not put the drone into simulation mode.
  * If you fail to do that second step, the real motors may be engaged once you start using the drone.
  * The simulator program simply takes data coming back from the flight controller (piped through the USB connection), simulates the flight, and sends fake sensor data back to the flight controller.
  	* The drone needs to be signalled to go into simulation mode, in which the flight controller takes its data from the simulation (instead of the sensors) and sends motor commands to the simulator rather the real motor controllers.
While in simulation, everything else works the same, so you can control the drone through the controller or code, and you should see the drone in the simulator respond.


## General Communication Pipeline
Your unity script --> DJI_SDK.cs --> ROSBridgeLib --> Network --> ROSBridge server (rone onboard computer) --> ROS --> Onboard SDK --> flight controller.


## Hardware Setup
This is a high level overview of the hardware setup - the specifics may vary.

### Drone
* Ensure that the drone supports API control (can be controlled through an onboad computer). Look on DJI's website and documentation for whether the drone supports this.
* Enable API control on the SDK tab in DJI Assistant 2.
  * Ensure that you have the correct version of DJI Assistant 2 installed. DJI Assistant 2 has different version for different lines of drones. Older drones may use DJI Assistant instead of DJI Assistant 2.
  * Some drones have special switches that change the operation mode of their USB port (the Matrice 210 series is one such example). Please refer to the manual to check that the drone is set in the correct mode in order for it to register/show up in DJI Assistant.
* Ensure that the controller is paired to the drone. The color LED's on the drone and controller will indicate whether it is connected (either through color or through flashing pattern).
* Refer to the manual for pairing instructions.

### Onboard Computer
An onboard computer should be installed on the drone. It is not necessary to have it mounted during development, as you will likely be "flying" the drone in simulated mode during development. You will need to mount it before actually flying it. Please refer to the model-specific drone manual for mounting instructions. You will need to connect the manifold to the drone itself through some wired interface (USB to UART, USB to TTL, etc)  
Please refer to the "Using the Manifold" document for instructions on setting up the Manifold.

### Other Computers (PCs and devices)
You will need several other "computers." These don't have to be separate computers, as you can run multiple of these things on the same computer. Often you can have DJI Assistant and Unity on the same machine.
* DJI Assistnt 2: this is the computer with DJI Assistant 2 installed on it. This is the main way to configure the settings on the drone itself, RTK activation, and run the simulation (more on this later).
* Unity: this computer runs tha actual Unity project that is being developed. This is the project that will be using this repositorie's code.
* (Optional) Second ROS Computer: while the main ROS instance (the one running the ROSBridge server) is the onboard computer, its often helpful to have a second linux machine with ROS for conveniently testing subscrbing to topics or performing service calls over ROSBridge.
* Mobile Device: This can be either an Android or iOS device with the correct verison of DJI Go installed (different models of drone use a different app).
  * Useful for drone activation and status monitoring. This is the main channel of communication for the drone to notify the user of any issues (eg. no/bad GPS signal, un-calibrated compass, vision system status, etc).
  * *Note: the first time an Onboard SDK API call is made from ROS, DJI will prompt to activate the drone through the mobile app.*
