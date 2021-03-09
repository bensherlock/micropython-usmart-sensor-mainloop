#! /usr/bin/env python
#
# MicroPython MainLoop for USMART Sensor Application.
#
# This file is part of micropython-usmart-sensor-mainloop
# https://github.com/bensherlock/micropython-usmart-sensor-mainloop
#
# Standard Interface for MainLoop
# - def run_mainloop() : never returns
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""MicroPython MainLoop for USMART Sensor Application."""

import pyb
import machine
import utime

from pybd_expansion.main.max3221e import MAX3221E
from pybd_expansion.main.powermodule import PowerModule
import sensor_payload.main.sensor_payload as sensor_payload

from uac_modem.main.unm3driver import MessagePacket, Nm3
from uac_modem.main.unm3networksimple import Nm3NetworkSimple

from uac_network.main.sensor_node import NetProtocol

import jotter

import micropython
micropython.alloc_emergency_exception_buf(100)
# https://docs.micropython.org/en/latest/reference/isr_rules.html#the-emergency-exception-buffer


_rtc_callback_flag = False
_rtc_alarm_period_s = 10
_rtc_next_alarm_time_s = 0


def rtc_set_next_alarm_time_s(alarm_time_s_from_now):
    global _rtc_next_alarm_time_s

    if 0 < alarm_time_s_from_now <= 7200:  # above zero and up to two hours
        _rtc_next_alarm_time_s = utime.time() + alarm_time_s_from_now
        print("_rtc_next_alarm_time_s=" + str(_rtc_next_alarm_time_s) + " time now=" + str(utime.time()))


def rtc_set_alarm_period_s(alarm_period_s):
    """Set the alarm period in seconds. Updates the next alarm time from now. If 0 then cancels the alarm."""
    global _rtc_alarm_period_s
    global _rtc_next_alarm_time_s
    _rtc_alarm_period_s = alarm_period_s
    if _rtc_alarm_period_s > 0:
        _rtc_next_alarm_time_s = utime.time() + _rtc_alarm_period_s
    else:
        _rtc_next_alarm_time_s = 0  # cancel the alarm

    print("_rtc_next_alarm_time_s=" + str(_rtc_next_alarm_time_s) + " time now=" + str(utime.time()))

_rtc_callback_seconds = 0  # can be used to stay awake for X seconds after the last RTC wakeup


def rtc_callback(unknown):
    # NB: You cannot do anything that allocates memory in this interrupt handler.
    global _rtc_callback_flag
    global _rtc_callback_seconds
    global _rtc_alarm_period_s
    global _rtc_next_alarm_time_s
    # RTC Callback function -
    # pyb.LED(2).toggle()
    # Only set flag if it is alarm time
    if 0 < _rtc_next_alarm_time_s <= utime.time():
        _rtc_callback_flag = True
        _rtc_callback_seconds = utime.time()
        _rtc_next_alarm_time_s = _rtc_next_alarm_time_s + _rtc_alarm_period_s  # keep the period consistent


_nm3_callback_flag = False
_nm3_callback_seconds = 0  # used with utime.localtime(_nm3_callback_seconds) to make a timestamp
_nm3_callback_millis = 0  # loops after 12.4 days. pauses during sleep modes.
_nm3_callback_micros = 0  # loops after 17.8 minutes. pauses during sleep modes.


def nm3_callback(line):
    # NB: You cannot do anything that allocates memory in this interrupt handler.
    global _nm3_callback_flag
    global _nm3_callback_seconds
    global _nm3_callback_millis
    global _nm3_callback_micros
    # NM3 Callback function
    _nm3_callback_micros = pyb.micros()
    _nm3_callback_millis = pyb.millis()
    _nm3_callback_seconds = utime.time()
    _nm3_callback_flag = True




# Standard Interface for MainLoop
# - def run_mainloop() : never returns
def run_mainloop():
    """Standard Interface for MainLoop. Never returns."""

    global _rtc_callback_flag
    global _rtc_callback_seconds
    global _nm3_callback_flag
    global _nm3_callback_seconds
    global _nm3_callback_millis
    global _nm3_callback_micros

    # Firstly Initialise the Watchdog machine.WDT. This cannot now be stopped and *must* be fed.
    wdt = machine.WDT(timeout=30000)  # 30 seconds timeout on the watchdog.

    # Now if anything causes us to crashout from here we will reboot automatically.

    # Last reset cause
    last_reset_cause = "PWRON_RESET"
    if machine.reset_cause() == machine.PWRON_RESET:
        last_reset_cause = "PWRON_RESET"
    elif machine.reset_cause() == machine.HARD_RESET:
        last_reset_cause = "HARD_RESET"
    elif machine.reset_cause() == machine.WDT_RESET:
        last_reset_cause = "WDT_RESET"
    elif machine.reset_cause() == machine.DEEPSLEEP_RESET:
        last_reset_cause = "DEEPSLEEP_RESET"
    elif machine.reset_cause() == machine.SOFT_RESET:
        last_reset_cause = "SOFT_RESET"
    else:
        last_reset_cause = "UNDEFINED_RESET"

    jotter.get_jotter().jot("Reset cause: " + last_reset_cause, source_file=__name__)

    print("last_reset_cause=" + last_reset_cause)


    # Feed the watchdog
    wdt.feed()


    # Set RTC to wakeup at a set interval
    rtc = pyb.RTC()
    rtc.init()  # reinitialise - there were bugs in firmware. This wipes the datetime.
    # A default wakeup to start with. To be overridden by network manager/sleep manager
    rtc.wakeup(10 * 1000, rtc_callback)  # milliseconds - # Every 10 seconds

    rtc_set_alarm_period_s(60 * 60)  # Every 60 minutes to do the sensors by default
    _rtc_callback_flag = True  # Set the flag so we do a status message on startup.

    pyb.LED(2).on()  # Green LED On

    jotter.get_jotter().jot("Powering off NM3", source_file=__name__)

    # Cycle the NM3 power supply on the powermodule
    powermodule = PowerModule()
    powermodule.disable_nm3()

    # Enable power supply to 232 driver and sensors and sdcard
    pyb.Pin.board.EN_3V3.on()
    pyb.Pin('Y5', pyb.Pin.OUT, value=0)  # enable Y5 Pin as output
    max3221e = MAX3221E(pyb.Pin.board.Y5)
    max3221e.tx_force_on()  # Enable Tx Driver

    # Feed the watchdog
    wdt.feed()

    jotter.get_jotter().jot("Powering on NM3", source_file=__name__)

    utime.sleep_ms(10000)
    powermodule.enable_nm3()
    utime.sleep_ms(10000)  # Await end of bootloader

    # Feed the watchdog
    wdt.feed()

    jotter.get_jotter().jot("NM3 running", source_file=__name__)

    # Set callback for nm3 pin change - line goes high on frame synchronisation
    # make sure it is clear first
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_DOWN, None)
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_DOWN, nm3_callback)

    # Serial Port/UART is opened with a 100ms timeout for reading - non-blocking.
    uart = machine.UART(1, 9600, bits=8, parity=None, stop=1, timeout=100)
    nm3_modem = Nm3(input_stream=uart, output_stream=uart)
    utime.sleep_ms(20)

    # nm3_network = Nm3NetworkSimple(nm3_modem)
    # gateway_address = 7

    # Grab address and voltage from the modem
    nm3_address = nm3_modem.get_address()
    utime.sleep_ms(20)
    nm3_voltage = nm3_modem.get_battery_voltage()
    utime.sleep_ms(20)
    print("NM3 Address {:03d} Voltage {:0.2f}V.".format(nm3_address, nm3_voltage))
    jotter.get_jotter().jot("NM3 Address {:03d} Voltage {:0.2f}V.".format(nm3_address, nm3_voltage),
                            source_file=__name__)

    # Sometimes (maybe from brownout) restarting the modem leaves it in a state where you can talk to it on the
    # UART fine, but there's no ability to receive incoming acoustic comms until the modem has been fired.
    # So here we will broadcast an I'm Alive message. Payload: U (for USMART), A (for Alive), Address, B, Battery
    alive_string = "UA" + "{:03d}".format(nm3_address) + "B{:0.2f}V".format(nm3_voltage)
    nm3_modem.send_broadcast_message(alive_string.encode('utf-8'))

    # Feed the watchdog
    wdt.feed()

    # Delay for transmission of broadcast packet
    utime.sleep_ms(500)

    # sensor payload
    sensor = sensor_payload.get_sensor_payload_instance()
    # sensor.start_acquisition()
    # sensor_acquisition_start = utime.time()
    # while (not sensor.is_completed()) and (utime.time() < sensor_acquisition_start + 5):
    #    sensor.process_acquisition()

    # Feed the watchdog
    wdt.feed()

    nm3_network = NetProtocol()
    nm3_network.init_interfaces(nm3_modem, sensor, wdt)  # This function talks to the modem to get address and voltage
    network_can_go_to_sleep = False

    utime.sleep_ms(100)

    # Feed the watchdog
    wdt.feed()

    # Uptime
    uptime_start = utime.time()



    # Turn off the USB
    # pyb.usb_mode(None)  # Except when in development

    # Operating Mode
    #
    # Note: Sensor data is only sent in response to a command from the gateway or relay.
    #
    # Sensor data is acquired when the RTC wakes us up. Default set RTC alarm to hourly.
    # In the absence of a packet with TTNF information we will at least continue to grab sensor values ready for when
    # we do next get a network packet.
    #
    # From Power up:
    #   WDT enabled at 30 seconds.
    #   NM3 powered. Sleep with wake on HW. Accepting unsolicited messages.
    #   Parse and feed to Localisation/Network submodules.
    #   RTC set to hourly to grab sensor data.
    #
    #   network.handle_packet() returns a stay awake flag and a time to next frame.
    #   If network says go to sleep with a time-to-next-frame
    #     Take off time for NM3 startup (7 seconds), set next RTC alarm for wakeup, power off NM3 and go to sleep.
    #     (Check TTNF > 30 seconds) - otherwise leave the NM3 powered.
    #
    #   If RTC alarm says wakeup
    #     power up NM3
    #     power up sensors and take a reading whilst NM3 is waking up.
    #
    #   General run state
    #     Go to sleep with NM3 powered. Await incoming commands and HW wakeup.
    #

    while True:
        try:

            # First entry in the while loop and also after a caught exception
            # pyb.LED(2).on()  # Awake

            # Feed the watchdog
            wdt.feed()

            # The order and flow below will change.
            # However sending packets onwards to the submodules will occur on receipt of incoming messages.
            # At the moment the RTC is timed to wakeup, take a sensor reading, and send it to the gateway
            # via Nm3 as simple unicast before returning to sleep. This will be expanded to accommodate the network
            # protocol with wakeup, resynchronise on beacon, time offset for transmission etc.

            # Start of the wake loop
            # 1. Incoming NM3 MessagePackets (HW Wakeup)
            # 2. Periodic Sensor Readings (RTC)

            # Enable power supply to 232 driver
            pyb.Pin.board.EN_3V3.on()

            if _rtc_callback_flag:
                _rtc_callback_flag = False  # Clear the flag
                print("RTC Flag. Powering up NM3 and getting sensor data." + " time now=" + str(utime.time()))
                jotter.get_jotter().jot("RTC Flag. Powering up NM3 and getting sensor data.", source_file=__name__)

                # Enable power supply to 232 driver and sensors and sdcard
                pyb.Pin.board.EN_3V3.on()
                max3221e.tx_force_on()  # Enable Tx Driver

                # Start Power up NM3
                utime.sleep_ms(100)
                network_can_go_to_sleep = False  # Make sure we keep NM3 powered until Network says otherwise
                powermodule.enable_nm3()
                nm3_startup_time = utime.time()

                # sensor payload - BME280 and LSM303AGR and VBatt
                sensor.start_acquisition()
                sensor_acquisition_start = utime.time()
                while (not sensor.is_completed()) and (utime.time() < sensor_acquisition_start + 5):
                    sensor.process_acquisition()
                    utime.sleep_ms(100)  # yield

                # Wait for completion of NM3 bootup (if it wasn't already powered)
                while utime.time() < nm3_startup_time + 7:
                    utime.sleep_ms(100)  # yield
                    pass

            # If we're within 30 seconds of the last timestamped NM3 synch arrival then poll for messages.
            if utime.time() < _nm3_callback_seconds + 30:
                if _nm3_callback_flag:
                    print("Has received nm3 synch flag.")

                _nm3_callback_flag = False  # clear the flag

                # There may or may not be a message for us. And it could take up to 0.5s to arrive at the uart.

                nm3_modem.poll_receiver()
                nm3_modem.process_incoming_buffer()

                while nm3_modem.has_received_packet():
                    # print("Has received nm3 message.")
                    print("Has received nm3 message.")
                    jotter.get_jotter().jot("Has received nm3 message.", source_file=__name__)

                    message_packet = nm3_modem.get_received_packet()
                    # Copy the HW triggered timestamps over
                    message_packet.timestamp = utime.localtime(_nm3_callback_seconds)
                    message_packet.timestamp_millis = _nm3_callback_millis
                    message_packet.timestamp_micros = _nm3_callback_micros

                    # Process special packets US
                    if message_packet.packet_payload and bytes(message_packet.packet_payload) == b'USMRT':
                        # print("Reset message received.")
                        jotter.get_jotter().jot("Reset message received.", source_file=__name__)
                        # Reset the device
                        machine.reset()

                    if message_packet.packet_payload and bytes(message_packet.packet_payload) == b'USOTA':
                        # print("OTA message received.")
                        jotter.get_jotter().jot("OTA message received.", source_file=__name__)
                        # Write a special flag file to tell us to OTA on reset
                        try:
                            with open('.USOTA', 'w') as otaflagfile:
                                # otaflagfile.write(latest_version)
                                otaflagfile.close()
                        except Exception as the_exception:
                            jotter.get_jotter().jot_exception(the_exception)

                            import sys
                            sys.print_exception(the_exception)
                            pass

                        # Reset the device
                        machine.reset()

                    # Send on to submodules: Network/Localisation UN/UL
                    if message_packet.packet_payload and len(message_packet.packet_payload) > 2 and \
                            bytes(message_packet.packet_payload[:2]) == b'UN':
                        # Network Packet

                        # Wrap with garbage collection to tidy up memory usage.
                        import gc
                        gc.collect()
                        (network_can_go_to_sleep, time_till_next_req_ms) = nm3_network.handle_packet(message_packet)
                        gc.collect()

                        print("network_can_go_to_sleep=" + str(network_can_go_to_sleep) + " time_till_next_req_ms=" + str(time_till_next_req_ms))
                        # Update the RTC alarm such that we power up the NM3 and take a sensor reading
                        # ahead of the next network frame.
                        if 60000 < time_till_next_req_ms:  # more than 60 seconds
                            # Next RTC wakeup = time_till_next_req_ms/1000 - 20
                            # to take into account the 10second resolution and NM3 powerup time
                            rtc_seconds_from_now = int((time_till_next_req_ms - 60000) / 1000)
                            rtc_set_next_alarm_time_s(rtc_seconds_from_now)
                            print("Set RTC alarm with rtc_seconds_from_now=" + str(rtc_seconds_from_now))
                            pass
                        else:
                            # RTC should default to hourly so leave alone.
                            print("Leaving RTC alarm as default (hourly).")
                            pass

                        # Check there's enough time to make it worth powering down the NM3
                        if network_can_go_to_sleep and time_till_next_req_ms < 30000:
                            # if not at least 30 seconds til next frame then we will not power off the modem
                            network_can_go_to_sleep = False

                        pass  # End of Network Packets

                    if message_packet.packet_payload and len(message_packet.packet_payload) > 2 and \
                            bytes(message_packet.packet_payload[:2]) == b'UL':
                        # Localisation Packet
                        pass  # End of Localisation Packets

            # If too long since last synch and not rtc callback and too long since
            if not _rtc_callback_flag and (utime.time() > _nm3_callback_seconds + 30):

                # Double check the flags before powering things off
                if (not _rtc_callback_flag) and (not _nm3_callback_flag):
                    print("Going to sleep.")
                    jotter.get_jotter().jot("Going to sleep.", source_file=__name__)
                    if network_can_go_to_sleep:
                        print("NM3 powering down.")
                        powermodule.disable_nm3()  # power down the NM3
                        pass

                    # Disable the I2C pullups
                    pyb.Pin('PULL_SCL', pyb.Pin.IN)  # disable 5.6kOhm X9/SCL pull-up
                    pyb.Pin('PULL_SDA', pyb.Pin.IN)  # disable 5.6kOhm X10/SDA pull-up
                    # Disable power supply to 232 driver, sensors, and SDCard
                    max3221e.tx_force_off()  # Disable Tx Driver
                    pyb.Pin.board.EN_3V3.off()  # except in dev
                    pyb.LED(2).off()  # Asleep
                    utime.sleep_ms(10)

                while (not _rtc_callback_flag) and (not _nm3_callback_flag):
                    # Feed the watchdog
                    wdt.feed()
                    # Now wait
                    # utime.sleep_ms(100)
                    # pyb.wfi()  # wait-for-interrupt (can be ours or the system tick every 1ms or anything else)
                    machine.lightsleep()  # lightsleep - don't use the time as this then overrides the RTC

                # Wake-up
                # pyb.LED(2).on()  # Awake
                # Feed the watchdog
                wdt.feed()
                # Enable power supply to 232 driver, sensors, and SDCard
                pyb.Pin.board.EN_3V3.on()
                max3221e.tx_force_on()  # Enable Tx Driver
                # Enable the I2C pullups
                pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
                pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up

            pass  # end of operating mode

        except Exception as the_exception:
            import sys
            sys.print_exception(the_exception)
            jotter.get_jotter().jot_exception(the_exception)
            pass
            # Log to file

    # end of while True

