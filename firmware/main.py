import network
import time
import tinypico as TinyPICO
import micropython
import esp32
import machine
import secret

from machine import Pin, PWM, ADC, TouchPad, RTC, UART


BUZZER_PIN = 21
BUTTON_PIN = 25
UART_RX_PIN = 32
UART_TX_PIN = 33
RESET_PIN = 5
SYNC_PIN = 18


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print("connecting to network...")
    wlan.connect(secret.ESSID, secret.PSK)
    while not wlan.isconnected():
        time.sleep(1)
print("network config:", wlan.ifconfig())


class Button:
    def __init__(self, pin, callback):
        self.pin = pin
        self.callback = callback

        self.enable()

    def _callback(self, pin):
        self.disable()
        self.callback(pin)

    def disable(self):
        self.pin.irq(trigger=0, handler=None)
        self._enabled = False

    def enable(self):
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self._callback)
        self._enabled = True

    @property
    def pressed(self):
        return not self.pin.value()

    @property
    def long_pressed(self):
        time.sleep(0.5)

        for i in range(5):
            time.sleep(0.1)
            if not self.pressed:
                return False

        return True

    @property
    def enabled(self):
        return self._enabled


class Buzzer:
    def __init__(self, pwm_pin):
        self.pwm_pin = pwm_pin

    def beep(self):
        self.pwm_pin.duty(512)
        time.sleep(0.05)
        self.pwm_pin.duty(0)

    def beeps(self, number):
        for i in range(number):
            self.beep()
            time.sleep(0.05)


class Bartendro:
    def __init__(self, uart, buzzer, button, reset_pin, sync_pin):
        self.uart = uart
        self.buzzer = buzzer
        self.button = button

        self.dispense_required = False
        self.last_dispense_ticks = None
        self.button_enable_delay_ms = 1000

        self.reset = Pin(reset_pin, Pin.OUT, value=1)
        # Ensure that the Bartendro has time to reset (high min. 1ms)
        time.sleep(0.1)
        self.reset.value(0)

        time.sleep(0.5)

        # self.sync = PWM(Pin(sync_pin), freq=500, duty=512)

        print("Waiting for dispenser")
        self.enter_text_mode()
        print("Bartendro ready!")

    def read(self):
        data = uart.read()
        for i in range(5):
            print("Reading (" + str(i) + "):")
            print(data)
            if data:
                break
            time.sleep(0.1)

        return data

    def enter_text_mode(self):
        while True:
            # In case the TinyPICO is reset, the Bartendro will already be
            # in text mode.
            prompt = self.read()
            if prompt == b"\r\nParty Robotics Dispenser at your service!\r\n\r\n>":
                break

            # Send text mode magic
            uart.write("!!!")
            time.sleep(2)

        # uart.write("led_idle\r")

    def dispense(self):
        if not self.dispense_required:
            self.dispense_required = True
            self.buzzer.beep()

    def run(self):
        if self.dispense_required:
            if self.button.long_pressed:
                # 50 200 => 7.5ml
                # 0.150ml per tick
                # 66 200 => 10ml
                #
                # 83 200 => 12.45ml (seems more like 11ml)
                uart.write("tickdisp 50 200\r")

                self.last_dispense_ticks = time.ticks_ms()
            else:
                self.buzzer.beeps(3)
                self.button.enable()

            self.dispense_required = False
        # Add a little delay after the dispense so the button release
        # bounces are not registered as a new dispense request
        elif not self.button.enabled and (
            time.ticks_diff(time.ticks_ms(), self.last_dispense_ticks)
            > self.button_enable_delay_ms
        ):
            self.button.enable()


buzzer = Buzzer(PWM(Pin(BUZZER_PIN), freq=4000, duty=0))
buzzer.beep()

uart = UART(1, baudrate=9600, tx=UART_TX_PIN, rx=UART_RX_PIN)

button = Button(
    Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP), callback=lambda pin: dispenser.dispense()
)

dispenser = Bartendro(uart, buzzer, button, RESET_PIN, SYNC_PIN)


while True:
    dispenser.run()
