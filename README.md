# red-crash

## Hacking

### udev rules
To make "nice" device names, put following in /etc/udev/rules.d/80-local.rules.  This will cause your devices to be listed under /dev/roboclaw and /dev/oakd
```
ATTRS{idVendor}=="03eb", , ATTRS{idProduct}=="2404", SYMLINK+="roboclaw"
ATTRS{idVendor}=="03e7", , ATTRS{idProduct}=="2485", SYMLINK+="oak-d"

```
then execute
```
udevadm control --reload
```
