import network
import time
import tinypico as TinyPICO
import micropython
import esp32
import machine
import blynklib_mp as blynklib
import blynklib_mp_ssl as blynklib_ssl
import secret

from machine import Pin, PWM, ADC, TouchPad, RTC, UART


BUZZER_PIN = 21
BUTTON_PIN = 25
UART_RX_PIN = 32
UART_TX_PIN = 33
RESET_PIN = 5
SYNC_PIN = 18

DISPENSE_BUTTON_VPIN = 0
COUNTER_VPIN = 4
RESET_COUNT_VPIN = 2
LOCK_CONTROLS_VPIN = 3
DISPENSE_GRAPH_VPIN = 5


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print("connecting to network...")
    wlan.connect(secret.ESSID, secret.PSK)
    while not wlan.isconnected():
        time.sleep(1)
print("network config:", wlan.ifconfig())


class Button:
    """
    Debounced pin handler

    usage e.g.:

    def button_callback(pin):
        print("Button (%s) changed to: %r" % (pin, pin.value()))

    button_handler = Button(pin=Pin(32, mode=Pin.IN, pull=Pin.PULL_UP), callback=button_callback)


    jedie/button_test.py
    """

    def __init__(self, pin, callback, trigger=Pin.IRQ_FALLING, min_ago=300):
        self.callback = callback
        self.min_ago = min_ago

        self._blocked = False
        self._next_call = time.ticks_ms() + self.min_ago

        pin.irq(trigger=trigger, handler=self.debounce_handler)

    def call_callback(self, pin):
        self.callback(pin)

    def debounce_handler(self, pin):
        if time.ticks_ms() > self._next_call:
            self._next_call = time.ticks_ms() + self.min_ago
            self.call_callback(pin)
        # else:
        #    print("debounce: %s" % (self._next_call - time.ticks_ms()))


class Bartendro:
    def __init__(self, uart, reset_pin, sync_pin):
        self.uart = uart

        self.dispense_required = False

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
        self.dispense_required = True

    def run(self):
        if self.dispense_required:
            uart.write("tickdisp 5 255\r")

            self.dispense_required = False

            # In packet mode we could poll PACKET_IS_DISPENSING but for now
            # all we can do is sleep or send a PR
            time.sleep(1)


def button_pressed(pin):
    print("Button " + str(pin) + " pressed")
    dispenser.dispense()


def lock_controls():
    global controls_locked

    blynk.virtual_write(LOCK_CONTROLS_VPIN, 1)
    controls_locked = True


def set_counter(count):
    global dispense_count

    dispense_count = count
    blynk.virtual_write(COUNTER_VPIN, dispense_count)
    blynk.virtual_write(DISPENSE_GRAPH_VPIN, dispense_count)


def inc_counter():
    set_counter(dispense_count + 1)


buzzer = PWM(Pin(BUZZER_PIN), freq=4000, duty=512)
time.sleep(0.05)
buzzer.duty(0)

uart = UART(1, baudrate=9600, tx=UART_TX_PIN, rx=UART_RX_PIN)

dispenser = Bartendro(uart, RESET_PIN, SYNC_PIN)

button = Button(
    Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP),
    callback=button_pressed,
    trigger=Pin.IRQ_FALLING,
)

ssl_connection = blynklib_ssl.SslConnection(secret.BLYNK_AUTH, port=443, log=print)
blynk = blynklib.Blynk(secret.BLYNK_AUTH, connection=ssl_connection)

# blynk = blynklib.Blynk(secret.BLYNK_AUTH)

controls_locked = None
dispense_count = None


@blynk.handle_event("write V" + str(LOCK_CONTROLS_VPIN))
def write_handler(pin, value):
    global controls_locked

    controls_locked = True if int(value[0]) == 1 else False

    print("Controls [{}]".format("LOCKED" if controls_locked else "UNLOCKED"))


@blynk.handle_event("write V" + str(COUNTER_VPIN))
def write_handler(pin, value):
    global dispense_count

    if dispense_count is None:
        dispense_count = int(value[0])
    else:
        blynk.virtual_write(COUNTER_VPIN, dispense_count)


@blynk.handle_event("write V" + str(DISPENSE_BUTTON_VPIN))
def write_handler(pin, value):
    if int(value[0]) == 1 and not controls_locked:
        dispenser.dispense()
        inc_counter()
        lock_controls()


@blynk.handle_event("write V" + str(RESET_COUNT_VPIN))
def write_handler(pin, value):
    if int(value[0]) == 1 and not controls_locked:
        set_counter(0)
        lock_controls()


for vpin in [COUNTER_VPIN, LOCK_CONTROLS_VPIN]:
    blynk.run()
    blynk.virtual_sync(vpin)

while True:
    blynk.run()
    dispenser.run()
