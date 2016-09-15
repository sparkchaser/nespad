# nespad
Use a NES controller as a GPIO-attached gamepad on a Raspberry Pi.

This driver allows the user to connect an unmodified classic NES controller to the GPIO pins
on a Raspberry Pi and use it as a joystick/gamepad.  The code is also intended to be useful
as a generic template for any digital controller that uses a shift register for reporting
button data (e.g., the SNES controller, or a homebrew joystick).

This is the software half of a project.  The hardware half will be posted to instructables.com
and a link will be added here.

One of the design goals of this driver was to produce sample code that can be re-used and
adapted easily, even for those without a lot of prior experience with the Linux kernel.
For that reason, this code includes a lot of comments and sacrifices raw efficiency for
the sake of being easier to understand.

### Compatibility
This code was designed for and tested on a Raspberry Pi 2 model B+, and should be compatible
with any generation 2 hardware.  If using this on a different generation board, please
double-check the pin numbers being used.

I have successfully built and tested this driver on the RetroPie distribution (Raspbian
based) and on Ubuntu MATE.  I have no reason to believe that it won't work on other distributions
as well, but I have not tested these myself.

### Building
In order to build the driver, you need the programs `gcc` and `make`.  You also need to locate
and install the package containing the kernel headers for the specific kernel version that
you're running.  For best results, make sure that you have installed all available updates
for your system and are running the latest available firmware.

To build the driver, simply run `make`.

To install the driver, run `sudo make install`.

**Note:** if your kernel version ends with a "+" character (check with `uname -r`), the driver
may get installed to the wrong location.  It's unclear why this happens, but there is a workaround.
_Before_ installing, create a symlink for your kernel's modules directory without the "+" in
the name.  For example,
```
$ uname -r
4.4.0-v7+
$ cd /lib/modules
$ ls -lF
drwxr-xr-x 3 root root 4096 Aug 17 12:02 4.3.6-v7+/
drwxr-xr-x 3 root root 4096 Aug 17 12:02 4.3.8-v7+/
drwxr-xr-x 7 root root 4096 Sep  1 13:49 4.4.0-v7+/
$ sudo ln -s 4.4.0-v7+ 4.4.0-v7
$ ls -lF
drwxr-xr-x 3 root root 4096 Aug 17 12:02 4.3.6-v7+/
drwxr-xr-x 3 root root 4096 Aug 17 12:02 4.3.8-v7+/
lrwxrwxrwx 1 root root   17 Sep 15 12:57 4.4.0-v7 -> 4.4.0-v7+/
drwxr-xr-x 7 root root 4096 Sep  1 13:49 4.4.0-v7+/
```
With this symlink in place, `make install` will store the driver in the correct location.

### Testing
Once the driver is loaded, a new joystick node should appear for the controller (for
example, `/dev/input/js0`).  To perform a quick test from the command line, run
`jstest /dev/input/js0`.  Any buttons pressed on the controller should be reported
on the console in real time.  If you are running a graphical desktop environment, then
you may have a graphical utility available for testing and configuring joysticks.

### Adapting this code for your purposes
TBD
