from dronekit import connect, VehicleMode
import time

def mavConnect():
    connection_string = 'tcp:127.0.0.1:5761'
    print("Connecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

if __name__ == '__main__':
    try:
        vehicle = mavConnect()
        print("Connected to vehicle")
        
        # Example: Print some vehicle attributes
        print("Vehicle mode: %s" % vehicle.mode.name)
        print("Global Location: %s" % vehicle.location.global_frame)
        print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
        print("Local Location: %s" % vehicle.location.local_frame)
        print("Attitude: %s" % vehicle.attitude)
        print("Velocity: %s" % vehicle.velocity)
        print("Battery: %s" % vehicle.battery)
        print("Last Heartbeat: %s" % vehicle.last_heartbeat)
        print("Is Armable?: %s" % vehicle.is_armable)
        print("System status: %s" % vehicle.system_status.state)
        print("Groundspeed: %s" % vehicle.groundspeed)    # settable
        print("Airspeed: %s" % vehicle.airspeed)    # settable

    except Exception as e:
        print("Error connecting to vehicle: %s" % str(e))
    finally:
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.close()
            print("Vehicle connection closed.")
