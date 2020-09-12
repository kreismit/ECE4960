<h4 style="background-color:LightGray" id="navbar"><a href="#L1">&emsp;Lab 1&emsp;</a><a href="#L2">&emsp;Lab 2&emsp;</a><a href="#L3">&emsp;Lab 3&emsp;</a><a href="#L4">&emsp;Lab 4&emsp;</a><a href="#L5">&emsp;Lab 5&emsp;</a><a href="#L6">&emsp;Lab 6&emsp;</a></h4>

<h1 id="L1"> Lab 1</h1>

## Goal

The purpose of this lab was to get acquainted with the [SparkFun Artemis Nano](https://www.sparkfun.com/products/15443) and with our custom Ubuntu VM.

## Materials

* 1 SparkFun Artemis RedBoard Nano
* 1 USB A-C cable
* 1 lithium-polymer rechargeable battery
* 1 laptop

## Procedure

### The IDE

Installed [Arduino from the Arch Linux repos](https://wiki.archlinux.org/index.php/Arduino). Only the IDE supports Artemis boards. `arduino-avr-core` is required for the IDE.

Installed the [Arduino core](https://github.com/sparkfun/Arduino_Apollo3) in the IDE:

* Open Tools > Board > Boards Manager in the IDE
* Search for apollo3
* Choose version 1.1.2 in the drop-down
* Click Install (this took several minutes)

Chose the "SparkFun RedBoard Artemis Nano" as the board.

### Testing

Uploaded four example sketches to test various parts of the board:

* Blink (from built-in examples)
* Example2_Serial (from SparkFun Apollo3 examples)
* Example4_analogRead (SparkFun)
* Example1_Microphone (SparkFun)

Also modified Example1_Microphone to blink the built-in LED when whistling.

## Results

### IDE

One pitfall was accessing the COM port, as Linux doesn't give users read-write permission by default. The permanent fix was a custom `udev` rule:
```shell
SUBSYSTEM=="tty" ATTRS{vendor}=="0x8086" ATTRS{subsystem_device}=="0x0740" MODE="0666"
```
Note that the exact requirements for this rule depend on the computer and the distro.

### Testing
The Blink example worked, blinking the blue LED labeled 19.
<video width="600" height="400"><source src="Lab1/Videos/Blink.mp4" type="video/mp4" controls></video>

The Serial example both input and output text via the USB serial connection.
<video width="600" height="400"><source src="Lab1/Videos/Serial.mp4" type="video/mp4" controls></video>

The analogRead sketch read temperature values which noticeably rose as I held my warm thumb against the device.
<video width="600" height="400"><source src="Lab1/Videos/Analog.mp4" type="video/mp4" controls></video>

The Microphone example showed that the loudest frequency doubled when I whistled an octave, indicating that the microphone is working well.
<video width="600" height="400"><source src="Lab1/Videos/Microphone.mp4" type="video/mp4" controls></video>

I added two pieces of code to the Microphone example to make it blink the LED when I whistle.

In `void setup()`:
```c++
pinMode(LED_BUILTIN,OUTPUT);
```
In `void loop()`:
```c++
if(ui32LoudestFrequency>800 && ui32LoudestFrequency<2000)
    digitalWrite(LED_BUILTIN,HIGH); // blink the LED when frequency is in whistling range
  else
    digitalWrite(LED_BUILTIN,LOW); // and not otherwise
```

This worked well. (It also picked up my squeaky chair or tapping on my desk.)
<video width="600" height="400"><source src="Lab1/Videos/WhistleDetect.mp4" type="video/mp4" controls></video>

While the Artemis Nano was plugged into my computer, plugging the battery in lit the yellow CHG light.

I commented out all the serial lines of code so that the board would not attempt to establish serial communication with my computer. Then, the board would recognize my whistle on battery too.

[See my code for Lab 1 here.](https://github.com/kreismit/ECE4960/tree/master/Lab1)

<h1 id="L2"> Lab 2</h1>

## Goal

## Materials

* 1 Artemis Nano
* 1 USB dongle
* 1 computer running the Ubuntu 18 VM


## Procedure

Downloaded the [distribution code](https://cei-lab.github.io/ECE4960/ece4960lab2dist.zip).
(Re)installed `bleak` using `pip install bleak` at the command line.
Downloaded the sketch `ECE_4960_robot` to the Artemis and opened the serial monitor at 115200 baud.
In the `ece4960lab2dist` folder, ran `./main.py` twice while the Artemis Nano was powered on to discover the board.

## Results and Lessons Learned

<image src="Lab2/bluetooth_discovery.png">

At first, VirtualBox saw neither the Bluetooth module in my laptop nor the USB dongle in the lab kit. However, my laptop's host OS saw both. After a quick web search, I found [this useful SuperUser post](https://superuser.com/questions/956622/no-usb-devices-available-in-virtualbox) which points out that users must be part of the `vboxusers` group or else no USB devices are accessible from VirtualBox. Adding my user to the group made the VM see both my laptop's Bluetooth radio and the USB dongle.

Another surprise I shouldn't have experienced was that the code didn't run when I executed `python main.py`. However, opening the file and noticing that it started with `#!/usr/bin/env python3` I realized it would run as a script, and it worked. This pointed out to me that `python` was mapped to `python2.7` and not `python3`. Always check versions.

