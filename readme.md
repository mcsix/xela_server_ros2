# ROS Node for XELA Sensors (ROS 2)

## Disclaimer

This Node is for ROS 2 and will not work with ROS 1. For ROS 1, please use the [ROS Node for XELA Sensors (ROS 1)](https://github.com/mcsix/xela_server_ros).

## License

This ROS 2 node package is provided as-is under the terms of the [MIT License](https://opensource.org/licenses/MIT).

### MIT License

```
MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Function overview

This node takes all information provided by the XELA Server application and broadcasts it via services and a publisher node to the ROS 2 network.

### Services
#### Service: xServStream
The `xServStream` service provided by the XELA Sensors ROS 2 node package allows users to request sensor data from the XELA Server application within the ROS network. This service can be used to retrieve information about one or more sensors connected to the XELA sensor array.

__Service Request__

* `sensor` (int): An integer representing the sensor number. Set to 0 to request data from all available sensors, or specify a specific sensor number (e.g., 1, 2, etc.) to retrieve data from a single sensor.

__Service Response__

The response from the xServStream service will contain information about the requested sensor(s) in the form of ROS 2 messages. Each sensor's data is encapsulated in a xela_server_ros2.msg.SensorFull message, which includes the following information:

* `message` (int): A unique identifier for the data message.
* `time` (float): The timestamp of the data message.
* `model` (str): The model name of the sensor.
* `sensor_pos` (int): The position or identifier of the sensor.
* `taxels` (list of `xela_server_ros2.msg.Taxel`): A list of taxel data, including their X, Y, and Z values.
* `forces` (list of `xela_server_ros2.msg.Forces`): A list of force data, including their X, Y, and Z values.

In case there is no valid data for the requested sensor(s), or if no sensor data is available at all when requesting data from all sensors, the response will contain a default sensor message with the model name set to "not_available", and no taxels nor forces present.

> Note: As only some sensors support forces, in case the user uses unsupported sensors, the forces list will be empty. The same applies if they have not enabled it in the `xServ.ini` file within the XELA Suite.

__Example Usage__

To test the xServStream service, you can use the ros2 service call command in a terminal. Here are some example usage scenarios:

* Request data from sensor 1:

```bash
ros2 service call /xServStream xela_server_ros2/srv/XelaSensorStream '{sensor: 1}'
```
* Request data from all available sensors (sensor number set to 0):

```bash
ros2 service call /xServStream xela_server_ros2/srv/XelaSensorStream '{sensor: 0}'
```
> Note: the `'{sensor: 0}'` can be omitted, as the default value for the sensor number is 0.

These commands will return the latest sensor data, and the response will include the details described above.
For an example, the following is the response of the sensor if there are no details about it:
```
xela_server_ros2.srv.XelaSensorStream_Response(
    data=[
        xela_server_ros2.msg.SensorFull(
            message=0, 
            time=0.0, 
            model='not_available', 
            sensor_pos=0, 
            taxels=[], 
            forces=[]
        )
    ]
)
```

In case there is valid data for the requested sensor or all sensors, the response will contain following details:
```
xela_server_ros2.srv.XelaSensorStream_Response(
    data=[
        xela_server_ros2.msg.SensorFull(
            message=1476, 
            time=1695690788.8365479, 
            model='uSPa44', 
            sensor_pos=1, 
            taxels=[
                xela_server_ros2.msg.Taxel(
                    x=47295,
                    y=14256,
                    z=34846
                ), 
                ...
            ], 
            forces=[
                xela_server_ros2.msg.Forces(
                    x=0.7007036209106445, 
                    y=4.3193559646606445, 
                    z=-4.425440788269043
                ), 
                ...
            ]
        ),
        ...
    ]
)
```
> Note: in case of specific sensor, the response will contain only one (the requested) sensor message in the list. There is no limit to how many sensors there can be.

### Topics / Publishers
#### Publisher: xServTopic

The `xServTopic` publisher within the XELA Sensors ROS 2 node package broadcasts sensor data to the ROS 2 network. This data is continuously published to the specified topic, and subscribers can listen to this topic to receive sensor information.

__Published Topic__
* Topic Name: `/xServTopic`
* Message Type: `xela_server_ros2.msg.SensStream`

__Published Data__
The data published by the `xServTopic` publisher consists of sensor information encapsulated in ROS 2 messages of type `xela_server_ros2.msg.SensStream`. Each message contains the following details:

* `sensors` (list of `xela_server_ros2.msg.SensorFull`): A list of sensor data, with each sensor's information defined as follows:
  * `message` (int): A unique identifier for the data message.
  * `time` (float): The timestamp of the data message.
  * `model` (str): The model name of the sensor.
  * `sensor_pos` (int): The position or identifier of the sensor.
  * `taxels` (list of `xela_server_ros2.msg.Taxel`): A list of taxel data, including their X, Y, and Z values.
  * `forces` (list of `xela_server_ros2.msg.Forces`): A list of force data, including their X, Y, and Z values.

__Example Usage__
Subscribers can listen to the `/xServTopic` topic to receive real-time sensor data. You can create ROS 2 nodes or applications that subscribe to this topic to access and process the sensor information.

By subscribing to this topic, users can access the latest data from all available sensors, making it a valuable resource for integrating XELA Sensors into ROS 2 applications.

Users can also access the stream in terminal with the following command:
```bash
ros2 topic echo /xServTopic
```

The response will look like the following:
```
sensors:
- message: 5548
  time: 1695696620.8967698
  model: uSPa44
  sensor_pos: 1
  taxels:
  - x: 49331
    y: 17509
    z: 37365
  ...
  forces:
  - x: -0.13371588289737701
    y: -0.10482559353113174
    z: 5.396903038024902
  ...
...
---
```

Each packet might contain several sensors, each of which contains several taxels and forces. Each will also contain message number, initial broadcast time and sensor position defined for better tracking.

## Components
### Messages
__xela_server_ros2.msg.SensorFull__
* `uint32 message` - shows the message number, used for tracking new updates
* `float64 time` - timestamp for when the message was generated (from server)
* `string model` - the model of the sensor, i.e. uSPa44
* `uint8 sensor_pos` - sensor position, stating which one it is in case there is any error with the list
* `Taxel[] taxels` - all taxels in a list, starting from top left and continuing line-by-line
* `Forces[] forces` - all forces, if available, can be empty

__xela_server_ros2.msg.Taxel__
* `int16 x` - X axis value for the taxel
* `int16 y` - Y axis value for the taxel
* `int16 z` - Z axis value for the taxel

__xela_server_ros2.msg.Forces__
* `float32 x`- calculated force on X axis (can be negative, depends on direction)
* `float32 y`- calculated force on Y axis (can be negative, depends on direction)
* `float32 z`- calculated force on Z axis

__xela_server_ros2.msg.SensStream__
* `SensorFull[] sensors`- list of all sensors

### Services
__xela_server_ros2.srv.XelaSensorStream__
* Inputs:
  * `uint8 sensor` - if none specified or set to 0, will force to return all sensors
* Outputs:
  * `SensorFull[] data` - depending on request, will contain 1 or more sensor objects


## Example code
### Service
```python
# import ROS 2 libraries and the Node
import rclpy
from rclpy.node import Node
# Import XELA Service type for the request
from xela_server_ros2.srv import XelaSensorStream


# Make a minimal client node
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # Create a client for the xServStream service with the correct XelaSensorStream type
        self.cli = self.create_client(XelaSensorStream, 'xServStream')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = XelaSensorStream.Request()

    def send_request(self, sensor):
        # Set the sensor number in the request
        self.req.sensor = sensor
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    # Define the main function, based on the ROS2 tutorial
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1)  # change tp the sensor needed or 0 for all
    log = minimal_client.get_logger()
    log.info(f'Result of XelaSensorStream: {response}')
    log.info(f'X of the first taxel of the first sensor returned: {response.data[0].taxels[0].x}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### Topic
```python
# import ROS 2 libraries and the Node
import rclpy
from rclpy.node import Node
# Import XELA Message type for the stream
from xela_server_ros2.msg import SensStream

# Make a minimal subscriber node
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscription to the xServTopic topic
        self.subscription = self.create_subscription(
            SensStream,
            'xServTopic',
            self.listener_callback,
            10)
    # Callback function for the subscription
    def listener_callback(self, msg):
        sensors = msg.sensors
        # Here we will just print out some details about the sensor, 
        # checking how many taxels has it reported and if it has forces included.
        log = self.get_logger()
        log.info("-------------------------")
        log.info(f"Broadcast: {len(sensors)} sensor(s)")
        for sensor in sensors:
            log.info(f"Model `{sensor.model}` at message `{sensor.message}` with " + 
		             f"{int(len(sensor.taxels)/3)} taxels" +
                     f" and {'with' if sensor.forces else 'without'} calibration")

# Define the main function, based on the ROS2 tutorial
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Installation
### Prerequisites
* [XELA Sensors](https://xelarobotics.com/) - XELA tactile sensors (hardware)
* [XELA Software](https://xela.lat-d5.com/) - XELA Software (1.7.6 or newer)
* [ROS 2](https://index.ros.org/doc/ros2/) - ROS 2 Foxy Fitzroy (or newer)
* [Python 3](https://www.python.org/) - Python 3.8 (or newer)

### Installation
1. Install the XELA software on your computer and add to your $PATH
2. Install Python 3 on your computer (if not done yet)
3. Install ROS 2 on your computer (if not done yet)
4. cd {your_ros2_workspace}/src
5. Clone this repository to your computer ```sh git clone https://github.com/mcsix/xela_server_ros2.git```
6. Run the following command to build the ROS 2 node package:
    ```bash
    colcon build --packages-select xela_server_ros2
    ```
7. Run the following command to source the ROS 2 node package:
    ```bash
    source install/setup.bash
    ```
8. Run the following command to start the ROS 2 node:
    ```bash
    ros2 run xela_server_ros2 xela_server_ros2
    ```
9. Set up your node or test out cli commands shown above

## Troubleshooting
### XELA Suite
* Make sure you have the latest version of the XELA Suite installed
* Make sure to follow instructions for its installation in the manual at [XELA Robotics (Support)](https://xela.lat-d5.com/)
* Make sure to check the troubleshooting section at the end of the manual and request support if needed, following 
  the guide at the end of the manual

### ROS 2
* Make sure you have sourced the ROS 2 environment and packages correctly
* Make sure you have the correct version of ROS 2 installed
* For support, open an issue in this repository or check the [ROS 2 documentation](https://docs.ros.org/en/rolling/)

### XELA Sensors ROS 2 Node
* Make sure you have followed the installation instructions correctly
* Make sure you have the correct version of Python installed
* Make sure you have the correct version of ROS 2 installed
* Make sure you have the correct version of the XELA Server installed
* Make sure the build and source commands have been executed correctly
* Make sure the node is running correctly
* For support, open an issue in this repository

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

### Contributors
| Photo | Contributor | Relation |
|:---:|:---:|:---:|
| [<img src="https://github.com/mcsix.png" width="40">](https://github.com/mcsix) | [@mcsix](https://github.com/mcsix) | Author |