# Useful ROS Commands

## Running ROS

* `roscore`: run master node
  * `roscore`

* `rosrun`: run node
  * `rosrun [package] [executable]`
    * `rosrun rqt_graph rqt_graph`: create node/publisher/subscriber graph
    * `rosrun rqt_plot rqt_plot`: creates a live graph of data being published  
    * `rosrun rqt_console rqt_console`: shows interface for viewing log messages
    * `rosrun rqt_logger_level rqt_logger_level`
  * `rosrun [package] [executable] __name:=[name]`

* rosbridge:
  * `roslaunch rosbridge_server rosbridge_websocket.launch`

* `rosnode`: operations on a nodes
  * `rosnode list`
    * `rosnode list -v`: includes the message type used by the topic
  * `rosnode info [node]`
  * `rosnode kill [node]`
  * `rosnode cleanup`
  * `rosnode ping`: checks that a rosnode is still alive

* `rostopic`: operations on topics.
  * `rostopic list`
  * `rostopic echo [topic]`
  * `rostopic info [topic]`
  * `rostopic hz [topic]`: publish rate (Hertz)
  * `rostopic bw [topic]`: bandwidth consumption
  * `rostopic pub -1 [topic] [message-type] [arguments]`: publishes message once
    * Hit TAB after typing message-type to get message template. Edit the template afterwards.
    * `--` is necessary before the [arguments] section if the arguments themsevles have dashes (eg. negative numbers)
  * `rostopic pub -l [rate] [topic] [message-type] [arguments]`: latched mode - ensures new subscribers will receive this message.
  * `rostopic pub -r [rate] [topic] [message-type] [arguments]`: rate mode - repeatedly sends message at certain rate.
  * `rostopic pub -f [topic] [message-type] [filename]`: read messages from file

* `rosservice`: operations on services.
  * `rosservice list`
  * `rosservice type [service-name]`: gives the message type of the service
  * `rosservice info [service-name]`
  * `rosservice node [service-name]`: finds the node that offers that service.
  * `rosservice call [service-name] ([arguments])`

* `rossrv`: inspecting service data types (operates on .srv files associated with services, not the services themselves)
  * `rossrv show [service datatype]`

* `rosbag`: recording messages and replaying them
  * `rosbag record -O [filename].bag [topics]`: records topics into bag.
    * Without `[filename].bag`, ROS will automatically name the file.
    * Replacing `[topics]` with `-a` will record every topic published.
    * Add `-j` for compression.
  * `rosbag play [filename].bag`: play the bag file back.
  * `rosbag info [filename].bag`

* `rosmsg`: operations on ros message type
  * `rosmsg show [message-type]`

* `rosparam`: dealing with parameters
  * `rosparam list`: lists out all parameter keys
  * `rosparam get [param_name]`: gets the value of parameter with key [param_name]
  * `rosparam set [param_name] [value]`: sets the value of parameter with key [param_name]
  * `rosparam dump [file_name] ([namespace])`: stores all current parameters into file. If namespace not provided then all namespaces are included.
  * `rosparam load [file_name] ([namespace])`: loads all current parameters into file. If namespace not provided then all namespaces are included.

## Dealing With Packages

* `ropsack`: operations on packages
  * `rospack list`
  * `rospack find [package]`
  * `rospack depends([n-level]) [package]`: Finds the dependencies of a package within the n'th level.
    * Performs a fully recursive search if [n-level] is not provieded.

* `rosls`: list directory of package
  * `rosls [package/subdirectories]`: lists content of package or package's subdirectory (with path relative to package root folder).

* `roscd`: change into package directory
  * `roscd [package]`
  * `roscd log`: changes directories to the folder where ROS logs are stored

* `catkin_create_pkg`: create a package. should be run in src directory of worksapce.
  * `catkin_create_pkg [package]`

* `rospack`:
  * `rospack find [package_name]`: finds the directory that encloses a package name

* `catkin_create_pkg`: creates package
  * `catkin_create_pkg [package_name] ([dependency1] [dependency2] ...)`

* `catkin_make`: builds the current workspace (current directory)
  * `catkin_make --source my_src`: usually it expects the source files to be in the /src folder of the workspace. If not then you can specify source files subdirectory using the `--source` option.
