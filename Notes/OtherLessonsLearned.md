# Bluetooth Communication

* Everything in `myRobotTasks()` runs **once**.
* The only functions which "loop" by default in the Python code are `checkMessages()` and `motorLoop()`. `motorLoop()` isn't enabled unless you `await asyncio.gather(checkMessages(), myRobotTasks(), motorLoop())`.
* Originally my approach was not to mess with `ece4960robot.py`, only with the "user-facing* code in the other Python scripts. But, as it turns out, there's a small bug in `self.updateFlag` in the `ece4960robot.py` code.
Initially, it's only set to true when a key is pressed on the keyboard. But I wasn't trying to drive the robot with my keyboard; I was trying to send motor values. So, when I set that flag to true within the updateMotor() function, I got the expected results when I called updateMotor() and motorLoop() was running.

# C Stuff I Wish I'd Known

## Pointers

I already knew [what a pointer is](https://www.cprogramming.com/tutorial/c/lesson6.html) (in a very basic sense) when I started the class, but I didn't understand what it meant to cast a pointer. It's just a memory address, right? Well, each variable stored in memory takes up a certain number of bytes. As Kirstin briefly explained in lecture, the type of a pointer indicates how many bytes after that address are allocated. This doesn't matter until you start allocating other memory and worrying about overwriting stuff that came after that variable. For that reason, the compiler won't let you use a pointer unless it has been typed properly.

The compiler WILL (unfortunately) let you use a pointer that points to a *system reserved address* between 0 and 255 (in hex, `0x00` and `0xff`). Those addresses are used for functions essential to the system, so you *really* don't want to overwrite data in them.

### How do I cast a pointer?

Easy. If you're casting a 32-bit unsigned integer, you'd type `uint32_t bar;`. A pointer is initialized with a type and then an asterisk, so the compiler knows it's a pointer. Imagine you're initializing `bar` again as a 32-bit variable, but just add a `*` after the type: `uint32_t* bar`.

### An array is a pointer.

There are two ways to dereference an array pointer: the same way you would another pointer (`*array`) or by accessing an entry in the array (`array[0]`). If you do it the first way, you will always get the first entry unless you add *a multiple of the size of the type* to the array pointer. Size in bytes, that is.

```c
int thingy;				// int has 8 bits = 1 byte
int* pointer = &thingy;	// address of thingy
double biggerThingy;	// double the memory: 2 bytes
// int* pointer = &biggerThingy;	// won't work
double* pointer2 = &biggerThingy;	// works
double array[25];		// EMPTY array with addresses 0 to 24
for(int i=0; i<25; i++){
	*(array+2*i) = i;		// dereferencing works on parenthetical expressions :)
	// array[i] = i;	// equivalent
	// *(array+i) = i;		// would only work if we had an array of 1-byte numbers
	// array->i = i;	// doesn't work
}

```

### A struct can be a pointer too.



# Making Ubuntu Work for You

## The `catkin_ws` Directory

Use links! Rather than copying all files you need into the folder, and editing them only on the VM, it's very convenient to have a file live in one place and link it in the `catkin_ws/src/labX/scripts` directory. Then you can open it from `/home/artemis` or from the shared folder instead. For example:

    ln -s /home/artemis/Shared/ECE4960/Lab9/commander.py commander.py

## Python 3.7

I needed to install the `python3.7-dev` package to get enough of Python to do what I wanted. Then, since ROS needed certain versions of Python, I changed all the links in `/usr/bin/` to point to `python3.7`:

    artemis@artemis-VirtualBox:~$ ls -lh $(which python3)

gave `/usr/bin/python3.6` after I installed; after I was done, it showed `/usr/bin/python3.7`. There were several other links I had to fix too:

```
artemis@artemis-VirtualBox:~$ ls -lh /usr/bin/python*
lrwxrwxrwx 1 root root    9 Apr 16  2018 /usr/bin/python -> python2.7
lrwxrwxrwx 1 root root    9 Apr 16  2018 /usr/bin/python2 -> python2.7
-rwxr-xr-x 1 root root 3.5M Sep 30 09:38 /usr/bin/python2.7
lrwxrwxrwx 1 root root   33 Sep 30 09:38 /usr/bin/python2.7-config -> x86_64-linux-gnu-python2.7-config
lrwxrwxrwx 1 root root   16 Apr 16  2018 /usr/bin/python2-config -> python2.7-config
lrwxrwxrwx 1 root root   18 Nov  9 13:56 /usr/bin/python3 -> /usr/bin/python3.7
-rwxr-xr-x 2 root root 4.4M Oct  8 08:12 /usr/bin/python3.6
lrwxrwxrwx 1 root root   33 Oct  8 08:12 /usr/bin/python3.6-config -> x86_64-linux-gnu-python3.6-config
-rwxr-xr-x 2 root root 4.4M Oct  8 08:12 /usr/bin/python3.6m
lrwxrwxrwx 1 root root   34 Oct  8 08:12 /usr/bin/python3.6m-config -> x86_64-linux-gnu-python3.6m-config
-rwxr-xr-x 2 root root 4.7M Nov  7  2019 /usr/bin/python3.7
lrwxrwxrwx 1 root root   33 Nov  7  2019 /usr/bin/python3.7-config -> x86_64-linux-gnu-python3.7-config
-rwxr-xr-x 2 root root 4.7M Nov  7  2019 /usr/bin/python3.7m
lrwxrwxrwx 1 root root   34 Nov  7  2019 /usr/bin/python3.7m-config -> x86_64-linux-gnu-python3.7m-config
lrwxrwxrwx 1 root root   16 Oct 25  2018 /usr/bin/python3-config -> python3.6-config
lrwxrwxrwx 1 root root   10 Aug 21 14:22 /usr/bin/python3m -> python3.6m
lrwxrwxrwx 1 root root   17 Oct 25  2018 /usr/bin/python3m-config -> python3.6m-config
lrwxrwxrwx 1 root root   16 Apr 16  2018 /usr/bin/python-config -> python2.7-config
artemis@artemis-VirtualBox:~$ sudo rm /usr/bin/python3m
artemis@artemis-VirtualBox:~$ sudo ln -s /usr/bin/python3.7m /usr/bin/python3m
artemis@artemis-VirtualBox:~$ sudo rm /usr/bin/python3-config 
artemis@artemis-VirtualBox:~$ sudo ln -s /usr/bin/python3.7-config /usr/bin/python3-config
artemis@artemis-VirtualBox:~$ sudo rm /usr/bin/python3m-config 
artemis@artemis-VirtualBox:~$ sudo ln -s /usr/bin/python3.7m-config /usr/bin/python3m-config
artemis@artemis-VirtualBox:~$ ls -lh /usr/bin/python*
lrwxrwxrwx 1 root root    9 Apr 16  2018 /usr/bin/python -> python2.7
lrwxrwxrwx 1 root root    9 Apr 16  2018 /usr/bin/python2 -> python2.7
-rwxr-xr-x 1 root root 3.5M Sep 30 09:38 /usr/bin/python2.7
lrwxrwxrwx 1 root root   33 Sep 30 09:38 /usr/bin/python2.7-config -> x86_64-linux-gnu-python2.7-config
lrwxrwxrwx 1 root root   16 Apr 16  2018 /usr/bin/python2-config -> python2.7-config
lrwxrwxrwx 1 root root   18 Nov  9 13:56 /usr/bin/python3 -> /usr/bin/python3.7
-rwxr-xr-x 2 root root 4.4M Oct  8 08:12 /usr/bin/python3.6
lrwxrwxrwx 1 root root   33 Oct  8 08:12 /usr/bin/python3.6-config -> x86_64-linux-gnu-python3.6-config
-rwxr-xr-x 2 root root 4.4M Oct  8 08:12 /usr/bin/python3.6m
lrwxrwxrwx 1 root root   34 Oct  8 08:12 /usr/bin/python3.6m-config -> x86_64-linux-gnu-python3.6m-config
-rwxr-xr-x 2 root root 4.7M Nov  7  2019 /usr/bin/python3.7
lrwxrwxrwx 1 root root   33 Nov  7  2019 /usr/bin/python3.7-config -> x86_64-linux-gnu-python3.7-config
-rwxr-xr-x 2 root root 4.7M Nov  7  2019 /usr/bin/python3.7m
lrwxrwxrwx 1 root root   34 Nov  7  2019 /usr/bin/python3.7m-config -> x86_64-linux-gnu-python3.7m-config
lrwxrwxrwx 1 root root   25 Nov  9 14:03 /usr/bin/python3-config -> /usr/bin/python3.7-config
lrwxrwxrwx 1 root root   19 Nov  9 14:03 /usr/bin/python3m -> /usr/bin/python3.7m
lrwxrwxrwx 1 root root   26 Nov  9 14:04 /usr/bin/python3m-config -> /usr/bin/python3.7m-config
lrwxrwxrwx 1 root root   16 Apr 16  2018 /usr/bin/python-config -> python2.7-config
artemis@artemis-VirtualBox:~$ 
```

Since Ubuntu 18.04 is two years old and officially supports only Python 3.6, I needed to use `pip` to install all the packages necessary. Some packages Ubuntu uses were no longer supported in 3.7! But I got everything working after installing these packages:

    jupyter-core
    jupyterlab
    rospkg
    rospy
    numpy
    scipy
    matplotlib
    pyqtgraph
    pyqt5

## Less Memory -> More Responsive

The Ubuntu 18.04 system, by default, has some significant memory leaks. Running the task manager (System Monitor) shows that the Evolution (Linux email) server is running in at least two processes and using near 100MB of memory. On a 2GB VM, this is too much. I followed [this tutorial](https://prahladyeri.com/blog/2017/09/how-to-trim-your-new-ubuntu-installation-of-extra-fat-and-make-it-faster.html) to disable these processes and free up RAM. It's old (2017) but it still works for Ubuntu 18 (released in 2018).

## The GNOME Desktop

### GNOME vs. Windows

If you're used to hitting the Windows key on your PC and typing stuff to run it, *exactly the same shortcuts will work on GNOME.* Hit the Windows key (Super, as it's called in Linux) and then the name of your desired app. If you don't remember what it's called, GNOME also recognizes generic names.

You can also split windows to the right and left of the screen using `Win+Right` and `Win+Left`, and maximize or minimize using `Win+Up` and `Win+Down` respectively.

Similarly, you can use `Alt+Tab` or `Win+Tab` to switch windows, like on MS Windows. The main thing is that the GNOME Overview replaces both the Start Menu and the Task View in Windows.

Yes, you can do virtual desktops in GNOME Overview exactly the same as you would in Windows. Drag windows from the overview to a desktop (on the upper right of the screen). The keyboard shortcut to switch virtual desktops is `Win+PgUp` or `Win+PgDn` instead of `Ctrl+Win+Right` or `Ctrl+Win+Left` because the desktops are virtually oriented.

### GNOME and Wayland

If you know about the X server, you will be interested to know that Ubuntu 18.04 does not use X anymore. It uses Wayland! So, GNOME is the only "official" desktop environment that works on Wayland. Others (like KDE-Wayland and Sway) aren't quite there yet as of the Ubuntu 18 release. So if you don't like GNOME (as a fair section of Linux users don't) you are unfortunately stuck with it.

### Customizing for Efficiency

But there are some tips and tricks to make it feel nicer. First, there's that bar on the side that's been in Ubuntu as long as I can remember. You can adjust the bar settings in the main Settings menu. Click "Applications" in the upper left-hand corner and you can type "Settings" to find the Settings app. Under Dock, you can reduce the icon size (and thus make the dock smaller and less obtrusive). You can also move it to the bottom or the right of the screen if you so desire. I really like the "Auto-hide the Dock" option which makes the dock go away whenever a window covers it. You can still see the dock when you drag the window away or when you go into the Overview.

Add `Screenshot` to the dock so you can easily screenshot your lab work. It's already installed - just right-click it when you find it in Overview and click "Add to Favorites".

### Customizing for Looks

I installed `gnome-tweak-tool` and `arc-gtk-theme`, and used the Tweak Tool to allow user themes and to load Arc. It looks nicer (I think) than the default.

You can also change other options in the Tweak Tool, including moving the window buttons to the left if you like the Mac OS layout better.

### If you have a touchscreen...

Firefox doesn't let you touch-scroll by default unless you set the environment variable `MOZ_USE_XINPUT2=1`. I tried setting the variable by adding a line to my `~/.bashrc`. However, this script only runs when the terminal application is run, so unless you start Firefox from a terminal window, touch scrolling still doesn't work. Actually, I had to add this line to the file `/etc/security/pam_env.conf`:

	MOZ_USE_XINPUT2 DEFAULT=1.

 The reason this is not set by default is that the environment variable isn't enabled for security reasons. 🤷

In the terminal, run `gsettings set org.gnome.settings-daemon.peripherals.mouse drag-threshold 24` to keep stuff from being dragged when you touch the screen intentionally or accidentally. The drag threshold, 24 pixels, is about the max it can reasonably be before intentional drags don't work.
I added this to my `~/.bashrc` file (just `nano` or `gedit .bashrc` in the terminal) to make it automatically change the drag threshold every time I log in. 

## .bashrc

The `.bashrc` file in the home directory is a script that is run every time you log into Ubuntu, as well as every time you run a terminal. The reason why you have to close all terminals after you set up a lab manager is that it sets aliases and environment variables in `~/.bashrc` and they won't be run until you restart the terminal.

In case you're reading this and wondering what all the symbols before `bashrc` mean: `~/` is short for `/home/artemis` (or `/home/`whatever user) and `.` means the file is hidden. If you `ls` a directory, you won't see hidden files unless you `ls -a`; similarly, in Files, you have to check "Show Hidden Files" to see them.

## Other useful terminal stuff

Hit `Ctrl+Alt+T` to open a terminal. This is a standard shortcut in Ubuntu and many other Linux distros.

Type `unzip` before a zipped filename in the terminal to extract it. You can also zip it back up in the terminal: this could be done like `zip bar.zip foo` where `bar.zip` is the output. This saves time when you're setting up a lab.

Type `clear` to clear (reset, clean up) the terminal.

I like to see what my machine is doing when it starts up. Just the splash screen isn't enough, especially if something goes wrong and you wonder why. I uninstalled `plymouth`, the Ubuntu splash screen program. Later, I edited `/etc/default/grub`, the Grub settings file, to remove `quiet` from the default command line:

    GRUB_CMDLINE_LINUX_DEFAULT="splash"

This is just a settings file. To reload the settings, I ran `sudo update-grub`.

Then, I saw the kernel output when the machine booted.

# Github Pages Site

## Links

When Github Pages builds a site from Markdown, it does *not* leave the pages in Markdown format. It turns them into HTML. So, when you link another page in your website, you *must* change the `.md` extension to `.html`.

## Editing Files on your Computer

For Linux users, I'd highly recommend a Markdown editor called [Marker](https://github.com/fabiocolacio/Marker). It has a live preview and lots of other cool features &mdash; I even use it to take class notes since it allows you to insert sketches &mdash; but yet it uses very little CPU or RAM.

On Windows or Mac, I've had a good experience with [Typora](https://typora.io). I switched to Marker because it was FOSS (Typora isn't open-source and I heard it will no longer be free to use soon.) But for now, you can [download Typora for free.](https://typora.io/#download)

## Including Images and Videos

Don't try to use native Markdown syntax for everything. You can easily [embed a video](https://www.w3schools.com/html/html5_video.asp) or [a picture](https://www.w3schools.com/html/html_images.asp) using the tutorials from W3Schools. An alternative is to embed YouTube videos.

## Headers and Stuff

You can add a header to all the pages by adding its code to `_layouts/default.html` in the Github directory where you're hosting the site (`/` or `/docs`). Make sure you download the correct `index.html` for your chosen theme and edit that, or else your site might look really funky.