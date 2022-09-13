# Bindings for BMA

## 支持的传感器

- BMA423

## 验证与测试

- [LILYGO T-Watch](bin/micropython-esp32-v1.19.bin)

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
```

#### 构造

##### bma.BMA423(i2c, address=24, int1=None, int2=None)

使用以下参数构造并返回一个新的 BMA423 对象：

| 参数 | 描述 |
| --- | ---- |
| i2c | machine.I2C对象句柄 |
| address | bma423传感器的i2c地址 |
| int1 | 中断1的Pin对象 |
| int2 | 中断1的Pin对象 |

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

##### BMA423.activity(handler, int_line=1)

设置活动检测的中断

| 参数 | 描述 |
| --- | ---- |
| handler | 活动检测的中断函数 |
| int_line | BMA423中断线 |

##### BMA423.single_tap(handler, int_line=1)

设置单击事件的中断

| 参数 | 描述 |
| --- | ---- |
| handler | 活动检测的中断函数 |
| int_line | BMA423中断线 |

##### BMA423.double_tap(handler, int_line=1)

设置双击事件的中断

| 参数 | 描述 |
| --- | ---- |
| handler | 活动检测的中断函数 |
| int_line | BMA423中断线 |

##### BMA423.wrist_wear(handler, int_line=1)

设置抬腕事件的中断

| 参数 | 描述 |
| --- | ---- |
| handler | 活动检测的中断函数 |
| int_line | BMA423中断线 |

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

##### INT1

BMA423中断线1

##### INT2

BMA423中断线2

## 开发计划

- [x] 步数统计
- [x] 活动检测中断
- [x] 单击中断
- [x] 双击中断
- [x] 抬腕中断
- [ ] 引入线程对中断进行处理
- [ ] 支持spi接口
- [ ] 更多的传感器
