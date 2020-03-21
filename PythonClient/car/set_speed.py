import airsim
import time

# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)

def set_speed(speed):
    '''
    Very basic function for controlling speed of car using APIs
    Can be improved through different control
    '''
    car_controls = airsim.CarControls()
    alpha = 0.01

    while(True):
        car_state = client.getCarState()
        print("Speed: {0}".format(car_state.speed))

        if car_state.speed < speed:
            car_controls.throttle += alpha
        elif car_state.speed > speed:
            car_controls.throttle -= alpha
        else:
            print("Achieved speed {0}, exiting".format(speed))
            return

        client.setCarControls(car_controls)

        # Sleep for some time
        time.sleep(0.01)

# Run car at 10m/s
set_speed(10.0)

client.enableApiControl(False)
