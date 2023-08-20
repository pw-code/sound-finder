sound-finder
====

This project is device that samples audio in the scene in front of it, and then draws the source of the sound on a screen.

There are 6 microphones arranged around the screen to help with triangulating were sounds are coming from.
The microphones are I2S devices, arranged in pairs (as left/right i2s channels).
We use a custom PIO program to sample these, using DMA, to 3 sets of buffers (each containing 2 mics as stereo pairs).
The buffers are double-buffered, so that we can analyse one sample, whilst another is being captured.
One core of the PICO is dedicated to analysing these audio samples to determine sound locations.

The camera is an OV7670 with a parallel interface. We use a custom PIO program to sample the video data, one row at a time.
Most of the PICO memory is taken up with audio buffers for sound analysis.
This means there is no room for a full video buffer. So instead we only hold a row of pixels.
Once core of the PICO read a row, adds overlays to show sound locations, and then sends the resulting new row to the LCD.
The camera output framerate is limited (todo: 15FPS?) so that we have time to process each row, and still keep a reasonable output rate.


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