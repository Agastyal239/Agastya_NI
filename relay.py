from gpiozero import OutputDevice
from time import sleep

relay = OutputDevice(2, active_high=True, initial_value=False)

try:
    while True:
        print("Relay ON")
        relay.on()
        sleep(3)

        print("Relay OFF")
        relay.off()
        sleep(3)

except KeyboardInterrupt:
    relay.off()
    print("Relay stopped")
