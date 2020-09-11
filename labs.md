<h3 style="color:White;background-image:linear-gradient(DarkSlateGray,DimGray)">&nbsp; <small>
<a href="./labs.html">&emsp;Labs&emsp;</a>&emsp;
<a href="https://github.com/kreismit/ECE4960/tree/master/Notes">&emsp;Class Notes&emsp;</a>&emsp;
<a href="https://github.com/kreismit/ECE4960/tree/master/">&emsp;Code&emsp;</a>
<a href="./index.html">&emsp;About&emsp;</a></small>
</h3>
<h4 style="background-color:LightGray"><a href="#Lab 1">&emsp;Lab 1&emsp;</a><a href="#Lab 2">&emsp;Lab 2&emsp;</a></h4>
# Lab 1

The purpose of this lab was to get acquainted with the [SparkFun Artemis Nano](https://www.sparkfun.com/products/15443) and with our custom Ubuntu VM.

## The IDE

Installing Arduino wasn't much of a challenge. Once I had all the right packages, I was actually able to use the official Arch repository, so I'll receive updates automatically.
One pitfall was accessing the COM port. This seems to be a typical problem for Linux users; the default `udev` rules did not give read-write permission to the user. A temporary workaround was `sudo chmod a+rw /dev/ttyUSB0`. The permanent fix was as follows:
```shell
SUBSYSTEM=="tty" ATTRS{vendor}=="0x8086" ATTRS{subsystem_device}=="0x0740" MODE="0666"
```
This custom `udev` rule was placed in the `/etc/udev/rules.d/` folder (location may vary for other users.)

After that, the program compiled, uploaded, and ran successfully.

<video width="600" height="400"><source src="Lab1/Videos/Blink.mp4" type="video/mp4" controls></video>
<video width="600" height="400"><source src="Lab1/Videos/Serial.mp4" type="video/mp4" controls></video>
<video width="600" height="400"><source src="Lab1/Videos/Analog.mp4" type="video/mp4" controls></video>
<video width="600" height="400"><source src="Lab1/Videos/Microphone.mp4" type="video/mp4" controls></video>
<video width="600" height="400"><source src="Lab1/Videos/WhistleDetect.mp4" type="video/mp4" controls></video>
# Lab 2
