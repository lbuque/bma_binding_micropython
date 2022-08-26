# Bindings for BMA

## 支持的传感器

- BMA423

## 编译指南

1. 获取源码

```shell
$ cd micropython
$ git clone https://github.com/liangyingy/bma_binding_micropython.git extmod/bma_binding_micropython
```

2. esp32

> 编译前请准备好esp-idf
>
> 关于esp32更详细的编译说，请参考https://github.com/micropython/micropython/tree/master/ports/esp32

```shell
$ cd ports/esp32/
$ make USER_C_MODULES=../../../extmod/bma_binding_micropython/micropython.cmake
```

## 使用指南

### class BMA423

BMA423对象用于控制BMA423传感器。可以获取传感器的加速度值，计步数等。

Usage Model:

```python
from machine import Pin, I2C
import bma
i2c = I2C(0, scl=Pin(22), sda=Pin(21),freq=400000)
b = bma.BMA423(i2c, 25)

# 配置使能三轴加速度
b.accel_config(True, direction=b.LOWER_LEFT, layer=b.BOTTOM_LAYER)
b.accel()

# 使能步数统计
b.step_config(True)
# 读取步数
b.step_counter()
```

#### 构造

##### bma.BMA423(i2c, address)

使用以下参数构造并返回一个新的 BMA423 对象：

| 参数 | 描述 |
| --- | ---- |
| i2c | machine.I2C对象句柄 |
| address | bma423传感器的i2c地址 |

#### 方法

##### BMA423.accel_config(enable, direction=LOWER_LEFT, layer=BOTTOM_LAYER)

使能三轴加速度，和配置传感器的位置信息。

| 参数 | 描述 |
| --- | ---- |
| enable | 使能失能重力加速度计 |
| direction | bma423传感器pin#1的标记位置 |
| layer | bma423传感器在PCB的位置 |


##### BMA423.accel()

获取三轴加速度值。

##### BMA423.x()

获取 x 轴值。

##### BMA423.y()

获取 y 轴值。

##### BMA423.z()

获取 z 轴值。

##### BMA423.temperature()

获取传感器内部温度值。

##### BMA423.reset()

软件复位传感器，所有的寄存器值恢复为默认值。但不清除步数值

##### BMA423.clear()

清除步数值。

##### BMA423.step_config(enable)

使能或者失能步数统计。

##### BMA423.step_counter()

读取步数。

#### 常量

##### BMA423.BOTTOM_LAYER

位于PCB底层

##### BMA423.TOP_LAYER

位于PCB顶层

##### UPPER_RIGHT

BMA423传感器Pin#1位于右上角

##### LOWER_LEFT

BMA423传感器Pin#1位于左下角

##### UPPER_LEFT

BMA423传感器Pin#1位于左上角

##### LOWER_RIGHT

BMA423传感器Pin#1位于右下角

## 开发计划

- [x] 步数统计
- [ ] 活动检测中断
- [ ] 单击中断
- [ ] 双击中断
- [ ] 支持spi接口
- [ ] 更多的传感器
