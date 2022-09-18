Introduction
=============

microPython module for bosch motion sensor driver.

Supported Sensors
==================

- BMA423

Compiling guide
================

Get the source code
--------------------

.. code-block:: shell

    cd micropython
    git clone https://github.com/liangyingy/bma_binding_micropython.git extmod/bma_binding_micropython


esp32
------

.. note::

    Please prepare esp-idf before compiling

    For more detailed compilation of esp32, please refer to https://github.com/micropython/micropython/tree/master/ports/esp32


.. code-block:: shell

    cd ports/esp32/
    make USER_C_MODULES=../../../extmod/bma_binding_micropython/micropython.cmake

Documentation
==============

API documentation for this library can be found on `Read the Docs <https://bma-binding-micropython-en.readthedocs.io/en/latest/>`_.

Verification and Testing
=========================

- [LILYGO T-Watch](bin/micropython-esp32-v1.19.bin)

Future plans
=============

- [x] 步数统计
- [x] 活动检测中断
- [x] 单击中断
- [x] 双击中断
- [x] 抬腕中断
- [ ] 引入线程对中断进行处理
- [ ] 支持spi接口
- [ ] 更多的传感器
