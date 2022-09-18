:mod:`bma` --- bosch motion sensor library
=============================================

.. module:: bma
    :synopsis: bosch motion sensor library

The module contains the relevant control methods for bosch motion sensors.

class BMA423
-------------

The BMA423 class is used to control the BMA423 sensor. It is possible to obtain the acceleration value of the sensor, count the number of steps, etc.

The BMA423 class is currently available as a Technical Preview. During the preview period, feedback from users is encouraged. Based on this feedback, the BMA423 class API and implementation may be changed.

For example::

    from machine import Pin, I2C
    import bma

    i2c = I2C(0, scl=Pin(22), sda=Pin(21),freq=400000)
    b = bma.BMA423(i2c, address=25, int1=Pin(39))

    # Configures to enable three-axis acceleration
    b.accel_config(True, direction=b.LOWER_LEFT, layer=b.BOTTOM_LAYER)
    b.accel()

    # Enable step count
    b.step_config(True)
    # The number of read steps
    b.step_counter()

    # Active detection interrupt
    def activity_cb(event):
        if event == 0:
            print('User is stationary')
        elif event == 1:
            print('User is walking')
        elif event == 2:
            print('User is running')
        elif event == 3:
            print('Invalid activity recognized')

    b.activity(activity_cb)

    # Click Event Break
    def single_tap_cb(event):
        print('Single tap received')

    b.single_tap(single_tap_cb)

    # Double-click the event interrupt
    def double_tap_cb(event):
        print('Double tap received')

    b.double_tap(double_tap_cb)

    # Suspension of the Arms Incident
    def wrist_wear_cb(event):
        print('Wrist wear received')

    b.wrist_wear(wrist_wear_cb)

Constructors
-------------
.. class:: BMA423(i2c, address=24, int1=None, int2=None)

    Construct and return a new BMA423 object with the following parameters:

        - *i2c*: machine.I2C objects
        - *address*: The i2c address of the bma423 sensor
        - *int1*: Pin object for receiving sensor interrupt 1
        - *int2*: Pin object for receiving sensor interrupt 2

Methods
--------

.. method:: BMA423.accel_configaccel_config(enable: bool, direction: int=LOWER_LEFT, layer: int=BOTTOM_LAYER)

    Enable three-axis acceleration, and configure the position information of the sensor.

        - *enable*: Enable Disabled Gravity Accelerometer
        - *direction*: The marked position of pin#1 of the BMA423 sensor
        - *layer*: The location of the BMA423 sensor on the PCB

.. method:: BMA423.accel()

    Get the three-axis acceleration value.

.. method:: BMA423.x()

    Get the x-axis value.

.. method:: BMA423.y()

    Get the y-axis value.

.. method:: BMA423.z()

    Get the z-axis value.

.. method:: BMA423.temperature()

    Get the temperature value inside the sensor.

.. method:: BMA423.reset()

    Software resets the sensor and all register values return to their default values. But the step value is not cleared.

.. method:: BMA423.clear()

    Clear the step value.

.. method:: BMA423.step_config(enable)

    使能或者失能步数统计。

.. method:: BMA423.step_counter()

    Read the number of steps.

.. method:: BMA423.activity(handler, int_line=1)

    Set interrupts for activity detection.

        - *handler*: Interrupt function for activity detection
        - *int_line*: BMA423 interrupt line

.. method:: BMA423.single_tap(handler, int_line=1)

    Set interrupt on click event.

        - *handler*: Interrupt function for activity detection
        - *int_line*: BMA423 interrupt line

.. method:: BMA423.double_tap(handler, int_line=1)

    Set interrupt on double click event.

        - *handler*: Interrupt function for activity detection
        - *int_line*: BMA423 interrupt line

.. method:: BMA423.wrist_wear(handler, int_line=1)

    Set the interrupt for the wrist raise event.

        - *handler*: Interrupt function for activity detection
        - *int_line*: BMA423 interrupt line

Constants
----------

.. data:: bma.BOTTOM_LAYER

    The BMA423 sensor is located on the bottom layer of the PCB

.. data:: bma.TOP_LAYER

    The BMA423 sensor is on the top layer of the PCB

.. data:: bma.UPPER_RIGHT

    BMA423 sensor Pin#1 is in the upper right corner

.. data:: bma.LOWER_LEFT

    BMA423 sensor Pin#1 is in the lower left corner

.. data:: bma.UPPER_LEFT

    BMA423 sensor Pin#1 is located in the upper left corner

.. data:: bma.LOWER_RIGHT

    BMA423 sensor Pin#1 is located in the lower right corner

.. data:: bma.INT1

    BMA423 interrupt line 1

.. data:: bma.INT2

    BMA423 interrupt line 2