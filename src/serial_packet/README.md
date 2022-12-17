# serial_packet

This package contains the node in charge of translating a
`[Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)`
topic to a custom message `SerialPacket` under `amp_msgs`.

## Nodes

#### packet_node.py

Translates a `Twist` topic denoted as `twist_topic` to a `SerialPacket` denoted
as `packet_topic`.

In its current state, the node is just scaling the linear velocity by 100 and
scaling the angular steering by 1000 and offsetting it by 3000.

**TODO**: Adequate scaling from `Twist` to `SerialPacket` will need to be tuned
and there may be a need to implement a feedback control mechanism to have the
kart correct to the values in `/cmd_vel`.

Params:

- `twist_topic` (string): Subscribed `Twist` topic.
- `packet_topic` (string): Subscribed `SerialPacket` topic.
