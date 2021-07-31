
`mavlink_receiver.cpp`

When a waypoint is sent to QGroundControl, the `MavlinkReceiver::handle_message(mavlink_message_t *msg)` receives a mavlink message with ID of `75` under `msg->msgid`.

The below line will trigger `MAVLINK_MSG_ID_COMMAND_INT`.
```cpp
case MAVLINK_MSG_ID_COMMAND_INT:
		handle_message_command_int(msg);
		break;
```

The below function is called, and the UORB message `vehicle_command_s vcmd{}` is called, and the mavlink message is passed further into the next function `handle_message_command_both`.
```cpp
void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
	vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	vcmd.param7 = cmd_mavlink.z;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = false;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}
```

The mavlink vehicle command is identified as `192` which represents `MAV_CMD_DO_REPOSITION`

```cpp
MAV_CMD_DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags.| Reserved| Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude| Longitude| Altitude|  */
```

Found how the `vehicle_command_s` is used **IF** `MAV_CMD_DO_REPOSITION` is called.

If is called in several places, but the most important place is `Navigator::run()` in `navigator_main.cpp`

```cpp

```
