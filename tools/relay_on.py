from RPi import GPIO


def main():
    reset_pin = 0
    RELAY_STATE_ON = 0
    RELAY_STATE_OFF = 1
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(reset_pin, GPIO.OUT)
    GPIO.output(reset_pin, RELAY_STATE_ON)


if __name__ == '__main__':
    main()
