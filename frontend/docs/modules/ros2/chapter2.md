---
sidebar_position: 3
title: Chapter 2 - Python Agents and ROS 2 Communication
---

# Chapter 2: Python Agents and ROS 2 Communication

## The Role of Python in Robotics and AI Integration

Python has become the dominant language for AI and robotics development due to its simplicity, extensive libraries, and strong community support. In the context of ROS 2, Python serves as the bridge between high-level AI algorithms and the robotic control system.

## Using rclpy to Create ROS 2 Nodes

The `rclpy` library is the Python client library for ROS 2. It provides the tools needed to create nodes, publish and subscribe to topics, and provide or call services.

### Creating Your First Node

Let's create a simple node that publishes sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publishing and Subscribing to Topics

In ROS 2, nodes communicate through topics using a publish-subscribe pattern. Let's look at how to create a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Calling and Exposing Services

Services provide a request-response communication pattern in ROS 2. Here's how to create a service server:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here's how to call that service:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging AI Logic to Motor and Sensor Control

One of the key challenges in robotics is connecting high-level AI decision-making with low-level motor and sensor control. This bridge is typically achieved through a multi-layered architecture:

1. **AI Layer**: High-level decision making
2. **Behavior Layer**: Translate decisions into robot behaviors
3. **Motion Layer**: Convert behaviors into specific movements
4. **Hardware Layer**: Direct control of actuators and sensors

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # For robot movement

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)

        # Publish movement commands
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        self.latest_sensor_data = None

    def sensor_callback(self, msg):
        self.latest_sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def ai_decision_callback(self):
        if self.latest_sensor_data:
            # Simple AI logic: if sensor detects obstacle, move backward
            if 'obstacle' in self.latest_sensor_data:
                self.move_backward()
            else:
                self.move_forward()

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.movement_publisher.publish(msg)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        msg = Twist()
        msg.linear.x = -0.5  # Move backward at 0.5 m/s
        self.movement_publisher.publish(msg)
        self.get_logger().info('Moving backward')

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIBridgeNode()
    rclpy.spin(ai_bridge)
    ai_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<details>
<summary>Advanced: Quality of Service (QoS) Settings</summary>

In production robotics, you may need to configure Quality of Service settings for different communication requirements:

- Reliability: Best effort vs. reliable
- Durability: Volatile vs. transient local
- History: Keep last N vs. keep all

These settings affect how messages are delivered and stored.
</details>

## Summary

In this chapter, you've learned how to use Python and rclpy to create ROS 2 nodes that can publish/subscribe to topics and call/expose services. You've seen practical examples of connecting AI logic to motor and sensor control, which is essential for creating intelligent robotic systems.

In the next chapter, we'll explore how robots are described in URDF and how this connects software to physical structure.