from RPi import GPIO


def main():
    reset_pin = 0
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(reset_pin, GPIO.OUT)
    GPIO.output(reset_pin, 0)


if __name__ == '__main__':
    main()
