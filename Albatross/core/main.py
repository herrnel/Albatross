from adapters.mavsdk_px4_adapter import MavlinkPX4Adapter, AGPEmulationConfig

adapter = MavlinkPX4Adapter(
    connection="udp:127.0.0.1:14540",
    emu=AGPEmulationConfig(
        start_pos_only=True,
        start_pos_duration_s=1.0,
        hide_imu=True,
        minimal_fields=True
    )
)

# Connect to the simulation
adapter.connect()

while True:
  
  # Implement the Adapter to read.
  

  
  
  # Implement a simple policy like fly forward
  
  
  # Implement the adapter to send