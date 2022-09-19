概况
====

用于博世运动传感器驱动程序的 microPython 模块。

支持的传感器
============

- BMA423

编译指南
========

获取源码
---------

.. code-block:: shell

    cd micropython
    git clone https://github.com/liangyingy/bma_binding_micropython.git extmod/bma_binding_micropython


esp32
------

.. note::

    编译前请准备好esp-idf

    关于esp32更详细的编译说，请参考 https://github.com/micropython/micropython/tree/master/ports/esp32


.. code-block:: shell

    cd ports/esp32/
    make USER_C_MODULES=../../../extmod/bma_binding_micropython/micropython.cmake

文档
=====

这个库的 API 文档可以在 `Read the Docs <https://bma-binding-micropython.readthedocs.io/zh_CN/latest/>`_ 中找到。

验证与测试
===========

- [LILYGO T-Watch](bin/micropython-esp32-v1.19.bin)

开发计划
========

- [x] 步数统计
- [x] 活动检测中断
- [x] 单击中断
- [x] 双击中断
- [x] 抬腕中断
- [ ] 引入线程对中断进行处理
- [ ] 支持spi接口
- [ ] 更多的传感器
