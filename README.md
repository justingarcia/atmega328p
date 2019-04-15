ATmega328P
-------------------------------------------------------------------------------
This is a series of small programs I wrote in order to learn AVR programming,
with the ultimate goal of building a device that would track my REM eye
movements through my eyelids (via IR LED/phototransistor pairs) while I was
dreaming. The device would softly notify me (via earphones) that I was dreaming
without waking me up, bringing me into a lucid dream. I would then be able to
communicate with the outside world using eye movements to navigate through an
audio UI and compose messages, which would be sent to a nearby computer and/or
the internet through Wi-Fi. The basic tracking setup was taken largely from
[this project](http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2013/msw234_sf323/msw234_sf323/msw234_sf323/Eyetracker.htm).

I made it as far as tracking eye movements with a crude prototype, whose code
can be found in
[`projects/ir/eyetrack2/eyetrack2.c`](projects/ir/eyetrack2/eyetrack2.c)
Many of the programs require firmware for the SimpleLink Wi-Fi CC3000 Module
from Texas Instruments, which can be obtained from
[Adafruit](https://github.com/adafruit/Adafruit_CC3000_Library). Also required
is the Petit Fat File System, which can be obtained
[here](http://elm-chan.org/fsw/ff/00index_p.html).
Finally, I had to write the firmware for the VS1053b Audio Codec Module from
scratch, and it can be found in [`vs1053`](vs1053).
