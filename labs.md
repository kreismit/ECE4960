<h3 style="color:White;background-image:linear-gradient(DarkSlateGray,DimGray)">&nbsp; <small>
<a href="./labs.html">&emsp;Labs&emsp;</a>&emsp;
<a href="https://github.com/kreismit/ECE4960/tree/master/Notes">&emsp;Class Notes&emsp;</a>&emsp;
<a href="https://github.com/kreismit/ECE4960/tree/master/">&emsp;Code&emsp;</a>
<a href="./index.html">&emsp;About&emsp;</a></small>
</h3>
<h4 style="background-color:LightGray"><a href="#Lab 1">&emsp;Lab 1&emsp;</a><a href="#Lab 2">&emsp;Lab 2&emsp;</a></h4>

# Lab 1

## Goal

The purpose of this lab was to get acquainted with the [SparkFun Artemis Nano](https://www.sparkfun.com/products/15443) and with our custom Ubuntu VM.

## Materials

* 1 SparkFun Artemis RedBoard Nano
* 1 USB A-C cable
* 1 lithium-polymer rechargeable battery
* 1 laptop (Arch Linux 5.8)

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

[See my code for Lab 1 here.](https://github.com/kreismit/ECE4960/tree/master/Lab1)

# Lab 2
