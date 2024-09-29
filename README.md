---
jupyter:
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
  language_info:
    codemirror_mode:
      name: ipython
      version: 3
    file_extension: .py
    mimetype: text/x-python
    name: python
    nbconvert_exporter: python
    pygments_lexer: ipython3
    version: 3.11.9
  nbformat: 4
  nbformat_minor: 2
---

::: {.cell .code execution_count="9"}
``` python
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
```
:::

::: {.cell .markdown}
# Bluetooth Thermometer Design Report
:::

::: {.cell .markdown}
This Notebook, documents the design process and considerationg in making
this bluetooth thermometer. The purpose of this project was to be an
exercise for PCB design in an IoT environment, and apply the basic
stages of product development as an engineer, and enable further
development in skills with electronics, DSP, digital design and embedded
systems.

### Product Summary

The thermometre device produce, takes a reading from an analogue
temperature sensor and broadcast\'s in an bluetooth low energy
advertising packet.Simultaneously, the temperature reading is displayed
a quantised temperature level across a row of RGB LED\'s which can have
the colour customised with a BLE connection to the thermometer itself.
\*\*\*
:::

::: {.cell .markdown}
## Product Planning

At this stage of development, the functional goals of the product are
set. This sets a frameworkfor testing the performance in the later
stages, and provides a target to work towards when iteratively improving
the systems within the product. The considerations were primarily
focused on the user interace with the device due it being the only
output of the system and on of the two inputs to the system, hence a
strong set of criteria was required.

### Requirements Analysis

  -----------------------------------------------------------------------
       Requirements             Constraints                Needs
  ----------------------- ----------------------- -----------------------
  The temperature can be     Power can only be    The device must be able
    determined visually   sourced from a battery  to have customisability
                             or a DC converted     for it to blend with
                          source (e.g USB, USB-C, its surroundings or to
                                   etc)            match user preference

    A alternative user    The device must have a   The device only needs
   interface can provide   ridid body so that it   to be supplied power
  user\'s a reading with   is robust and can be       for it to start
     an acccuracy of 2            mounted         working, with no setup
      degrees Celsius                                    involved

    The Product must be                           
   under \$30 to produce                          
  -----------------------------------------------------------------------

### Problem Statement

The problem is to design a product that will assist users to determine
the ambient room temperature. The product should give users multiple
options to read the temperature, that can be selected based on the
effort levels. The product should allow users to customise display
methods based on there needs and allow for the product to blend with
surrounding home decor. The product may offer many features, however,
the central challenge is to measure the temperature and communicate it
to a user in different formats at the same time. \*\*\*
:::

::: {.cell .markdown}
## Conceptualisation

At this stage, the thermometer\'s main components and aspects are
selected and given detail to assist the detailed design stage. The focus
is the overall functions of the product and how that will translate to
hardware/software for implementation.

### Concept

The produce developed will have an LED strip that can show the
temperature with the LED\'s colour arrangement along the strip, similar
to that of a mercury thermometer.The colour of the LED\'s will be
customisable to match a surrounding colour theme or the user\'s
preference.

The interface to change the internal settings of the device, will occur
over a wireless domain(Bluetooth LE) to provide a low effort requirment
and to meet the standards of the 21st century. Since a wireless
connection will be required to configure the settings, an efficient use
of this domain would be to also communicate the temperature reading with
a numerical format. This will allow for a greater accuracy in the
reading that will originate from the sensor subsytem itself.

The device will be powered by a wired connection, with a common USB-Type
C Connector to supply DC power to the device. This was due to the higher
power consumption of LEDs which would drain the battery rather quickly,
hence a USB-C provides a highly accessible option for users to connect.

### High Level Design

As previously discussed the product will be supplied by a USB type-C
port to a microcontroller that can interface over bluetooth low energy
and via digital signalling with the other subsystems. Each of the other
subsytems will be powered by the same USB-C source which will be set to
the default 5V supply.

The temperature sensor will be an analogue sensor from which an ADC will
sample it and use this data to then configure the LEDs and advertising
packet. The LEDs will be an RGB type to allow for colour changes and
contrast to be displayed. A diagram of this is shown below.

```{=html}
<center>
```
`<image src= "HLD.png" width=45%>`{=html}`</image>`{=html}`<center>`{=html}

------------------------------------------------------------------------
:::

::: {.cell .markdown}
## Hardware Design

### LEDs

The LED selected was the WS2182b. This was selected for its programmable
RGB combination and brightness. The WS2182B will be able to take as an
input a 24 bit PCM coded stream of data to reflect each of the RGB level
for each LED. After the first packet is taken the following packet is
transmitted to the next LED. This will allow for the microcontroller to
be able to change the colour and brightness of each individual LED
without requiring 24 PWM channels continously running. Hence this method
is an efficient use of the MCU without a large range of PWM peripherals
or an LED controller. Since the signally for the LED\'s is above 3.3V a
level shifter will be required for signally to be shifted and sent to
the LED\'s effectively.

### Microcontroller

The nrf52840 was chosen due to its low cost and detailed documentation
available. The Nordic Semiconductor SiC contains an Arm Cortex M4
(64Mhz) CPU with an integrated 2.4 Ghz Transceiver and Radio Controller
that can be easily intefaced with software. This will enable the device
to have firmware easily implement and control the BLE transmission mode
(2Mbps ,1Mbps etc), along with the bluetooth profile of the device.

The SiC has a 4 Channel PWM Generator that is integrated with DMA for
fast and effient signalling. This feature will be used along with the
internal PPI(Peripheral Peripheral Interconnect)(Figure 2) to signal to
WS2812b LED\'s and control the sequence period when it is updated.

The NRF52840 has a 12bit ADC placed internally with a maximum sampling
rate of 200ksp/s and a variable analogue gain channel to customise the
quantisation level size of the ADC. This function will be used to place
the voltage range of the analogue temperature sensor to the gain level
required allowing for an optimised resolution. This adc will directly
sample the temperature sensor connected to the pin.

### Power Supply

The power supply chosen was an LDO (TLV2117) to ensure a stable and low
noise 3.3V source was supplied to the NRF52840, this was chosen due to a
USB type-C power source having a high noise level. However, since the
WS2812B LED\'s are a have a high voltage range and are a digital system,
the noise level directly from the USB Type-C 5V suppy will suffice to
power them.

### Sensor

The TMP235A was selected as the temperature sensor to be used for its
simplicity and low cost. The accuracy of the sensor is $\pm2\degree C$,
which will match the criteria identified from the requirements
analysis.This sensor can be supplied directly from 3.3V. However,
following the data sheet recommendation a 0.1 $\mu F$ decoupling
capacitor will be connected to reduce the amount of digital noise that
couples to the analogue sensor output. The sensor utilises thermal
diodes to produce a voltage variation proportional to the temperature.
The output of the TMP236 is a buffered output meaning with an output
driver designed for direct connection to ADC\'s and can supply a 1000pF
load, hence an interfacing circuit is not required. The specification of
the TMP236 specifies for the temperature region being considered the
voltage output is $19.5mV / \degree C$ with an offset voltage of 400mV.
This can also be written as a linear equation as shown below:
$$V = 0.0195T + 0.4$$

### PCB Layout Considerations

Key Considerations in the layout of this design were to avoid EMI and
transmissoin line effects. Due to the board shape being a long board and
each LED being spread evenly along the long dimension of the board.

------------------------------------------------------------------------
:::

::: {.cell .markdown}
## Firmware Development

### Firmware Structure

The firmware has been written with the use of the nordic NRFx Drivers
and the Zephyr real time operating system. The Zephyr RTOS will create
and maintain the BLE stack, whilst also allowing for easy creation of
threads and memory for efficient power usage. The structure of this
firmware contains two main threads, one to manage the notifications and
LED signalling of the device and the other ot manage the filtering and
ADC use of the device. The Data will be passed between the two threads
in a threadsafe manner with the use of a message que. A messsage Que is
a simple linked list data structure thats is used to pass a specific
struct containing data, between threads. The overall strucutre is
illustrated below:

### BLE Configuration

The configuration used for the bluetooth low energy model is a server
connectable device. The GATT model includes the environmental sensing
primary servie with a temperature charactiristic and a client
configuration descriptor within it. This is so that the device can
connect and enable regular updates remotely.

### LEDs Signalling

The LEDs chosen are the WS2812B\'s which have there own RZ (Return to
Zero) communication protocol shown in the first figure below. To enable
this protocol to effictively the PWM drivers need3e to be used for a
custom duty cycle (since this is where the information is stored). In
addition to the PWM drivers, a PPI bus connected with a timer to allow
for data to be segmented into 24 bit packets to address each LED. The
PPI enables the timer and PWM to be reset without a direct CPU
intervention.

`<img display="flexible" align="right" src= "WS2812B.png" width="30%">`{=html}`</img>`{=html}

  Symbol   Timing
  -------- ------------
  T0H      0.4 us
  T1H      0.8 us
  T0L      0.85 us
  T1L      0.45 us
  Res      $\ge$ 50us

The individual LED\'s each will recieve a packet of 24 bits with 8 bits
dedicated to the intensity of each RGB component send by .The WS2812B
will store the value of the packet last received also allowing for the
LED\'s to remain unchanged until they are updates with a new packet.
This will allow for further power saving since LED signalling will only
need to occur at a custom rate and can be slowed down to reduce overal
power consumption of the device. With this method the default colours of
contrast chosen were red and blue to allow for a clear indication to
begin.

### ADC Sampling

The ADC peripheral in the nRF52840 is a Successive Approximation ADC
that has a adjustable resolution and gain. The resolution was set to
12bit with a 2 hz sampling rate in a blocking method due to the low
chance that the output will change at high speed. The ADC sampling and
conversion is completed in a seperate thread and sent over a workque to
the bluetooth and lighting thread to ensure maximum power efficiency.
However due to this multithreaded approach the sampling rate will not be
precisely 2Hz but have a small variation due to the surrounding
instrucutions however this will have minimal magnitude in comparison to
the sampling period when the CPU uses a 64Mhz clock.

The conversion is made by the CPU by first making the data conversion to
mV with the formula:
$V = Sample \cdot\frac{Gain}{Reference} \cdot \frac{1}{2^{12}}$

### Digital Filter Design
:::
