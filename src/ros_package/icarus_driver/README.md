# ICARUS DRIVER

#### API of Icarus driver


| NAME  | DESCRIPTION | FUNCTION |
| -------------| ------------- | ------------- |
| set mode  | set the mode you want to operate with the drone  |`IcarusDriver::SetMode(std::string mode)`|
| arm / disarm  | arm or disarm the drone | `IcarusDriver::armDisarm(int arm)`|
| take off  | take off to a preorder altitude  |`IcarusDriver::takeoff(double alt)`|
| move to local point  | move to a point in the local map |`IcarusDriver::moveLocalTo(double x, double y,double z)`|
| ack notifier | notify an external serial port device |`IcarusDriver::notifyAck(std::string msg)`|


> TODO: landing(), emergency_landing(), move_global_to()

# BOCANEGRA

Bocanegra is a state machine conceived specially for this project and addressed to give the best performace for our purpose.


<p align="center">
  <img width="550" height="340" src="docs/bocanegra_diagram.png">
</p>

#### API of Bocanegra state machine driver

| NAME  | DESCRIPTION | FUNCTION |
| -------------| ------------- | ------------- |
| is active | check if a node is active  | `Bocanegra::isActive(std::string node_name)` |
| activate  | active an inactivate node | `Bocanegra::activate(std::string node_name)`|
| deactivate  | deactivate a node |`Bocanegra::deactivate(std::string node_name)`|
| states callback | give the vector of states |`Bocanegra::statesCallback(const boca_negra_msgs::states::ConstPtr msg)`|



#### Bocanegra messages


| NAME  | DESCRIPTION | MSGS |
| -------------| ------------- | ------------- |
| node_state | msg with the node and its state |<p>`std_msgs/String node_name`</p> <p>`std_msgs/Bool is_active`</p>|
| states  | array with all the current node_states  | `state[] array`|


#### Bocanegra services


| NAME  | DESCRIPTION | SRV |
| -------------| ------------- | ------------- |
| state_change | <p>request: node_name + state</p><p>response: node_name + new state</p>|<p>`std_msgs/String node_name`</p>`std_msgs/Bool active`<p>---</p><p>`std_msgs/String node_name`</p><p>`std_msgs/Bool active`</p>|


#### EXAMPLE

<p align="center">
  <img width="580" height="310" src="docs/bocanegra_example.png">
</p>
