:mod:`bma` --- 运动传感器
==========================

.. module:: bma
    :synopsis: bosch 运动传感器

该模块包含 bosch 运动传感器的相关控制方法。

class BMA423
-------------

BMA423 类用于控制BMA423传感器。可以获取传感器的加速度值，计步数等。

BMA423 类目前作为技术预览版提供。 在预览期间，鼓励用户提供反馈。 根据此反馈，BMA423 类 API 和实现可能会发生变化。

例如::

    from machine import Pin, I2C
    import bma

    i2c = I2C(0, scl=Pin(22), sda=Pin(21),freq=400000)
    b = bma.BMA423(i2c, address=25, int1=Pin(39))

    # 配置使能三轴加速度
    b.accel_config(True, direction=b.LOWER_LEFT, layer=b.BOTTOM_LAYER)
    b.accel()

    # 使能步数统计
    b.step_config(True)
    # 读取步数
    b.step_counter()

    # 活动检测中断
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

    # 单击事件中断
    def single_tap_cb(event):
        print('Single tap received')

    b.single_tap(single_tap_cb)

    # 双击击事件中断
    def double_tap_cb(event):
        print('Double tap received')

    b.double_tap(double_tap_cb)

    # 抬腕事件中断
    def wrist_wear_cb(event):
        print('Wrist wear received')

    b.wrist_wear(wrist_wear_cb)

Constructors
-------------
.. class:: BMA423(i2c, address=24, int1=None, int2=None)

    使用以下参数构造并返回一个新的 BMA423 对象：

        - *i2c*: machine.I2C 对象句柄
        - *address*: bma423 传感器的 i2c 地址
        - *int1*: 用于接收传感器中断1的 Pin 对象
        - *int2*: 用于接收传感器中断1的 Pin 对象

Methods
--------

.. method:: BMA423.accel_configaccel_config(enable: bool, direction: int=LOWER_LEFT, layer: int=BOTTOM_LAYER)

    使能三轴加速度，和配置传感器的位置信息。

        - *enable*: 使能失能重力加速度计
        - *direction*: BMA423 传感器 pin#1 的标记位置
        - *layer*: BMA423传感器在 PCB 的位置

.. method:: BMA423.accel()

    获取三轴加速度值。

.. method:: BMA423.x()

    获取 x 轴值。

.. method:: BMA423.y()

    获取 y 轴值。

.. method:: BMA423.z()

    获取 z 轴值。

.. method:: BMA423.temperature()

    获取传感器内部温度值。

.. method:: BMA423.reset()

    软件复位传感器，所有的寄存器值恢复为默认值。但不清除步数值。

.. method:: BMA423.clear()

    清除步数值。

.. method:: BMA423.step_config(enable)

    使能或者失能步数统计。

.. method:: BMA423.step_counter()

    读取步数。

.. method:: BMA423.activity(handler, int_line=1)

    设置活动检测的中断。

        - *handler*: 活动检测的中断函数
        - *int_line*: BMA423 中断线

.. method:: BMA423.single_tap(handler, int_line=1)

    设置单击事件的中断

        - *handler*: 活动检测的中断函数
        - *int_line*: BMA423 中断线

.. method:: BMA423.double_tap(handler, int_line=1)

    设置双击事件的中断

        - *handler*: 活动检测的中断函数
        - *int_line*: BMA423 中断线

.. method:: BMA423.wrist_wear(handler, int_line=1)

    设置抬腕事件的中断

        - *handler*: 活动检测的中断函数
        - *int_line*: BMA423 中断线

Constants
----------

.. data:: bma.BOTTOM_LAYER

    BMA423 传感器位于 PCB 底层

.. data:: bma.TOP_LAYER

    BMA423传感器位于PCB顶层

.. data:: bma.UPPER_RIGHT

    BMA423 传感器 Pin#1 位于右上角

.. data:: bma.LOWER_LEFT

    BMA423 传感器 Pin#1 位于左下角

.. data:: bma.UPPER_LEFT

    BMA423 传感器 Pin#1 位于左上角

.. data:: bma.LOWER_RIGHT

    BMA423 传感器 Pin#1 位于右下角

.. data:: bma.INT1

    BMA423中断线1

.. data:: bma.INT2

    BMA423中断线2
