from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def mavConnect():
    connection_string = 'tcp:127.0.0.1:5761'
    print("Connecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def return_to_launch(vehicle):
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

if __name__ == '__main__':
    try:
        vehicle = mavConnect()
        arm_and_takeoff(vehicle, 3)

        # Hover for 10 seconds
        print("Hovering for 10 seconds")
        time.sleep(10)

        # Return to launch
        return_to_launch(vehicle)

        # Wait until the vehicle reaches the launch point
        while vehicle.location.global_relative_frame.alt > 1:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            time.sleep(1)

        print("Landed successfully")

    except Exception as e:
        print("Error: %s" % str(e))
    finally:
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.close()
            print("Vehicle connection closed.")
