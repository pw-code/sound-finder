sound-finder
====


# Building
```
$ mkdir build
$ cd build
$ cmake ..
```
then
```
$ make
```

# Loading into the pico

Force into BOOTSEL mode and only flash changed pages
```
picotool load -u sound-finder.bin -f
```

View USB serial output
```
minicom -o -D /dev/ttyACM0
```