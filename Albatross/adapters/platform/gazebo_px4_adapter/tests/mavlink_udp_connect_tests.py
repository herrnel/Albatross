import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            break

    # wait until PX4 says it's armable
    print("Waiting until armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Armable!")
            break

    await drone.action.arm()
    print("✅ Armed!")

if __name__ == "__main__":
    asyncio.run(main())
