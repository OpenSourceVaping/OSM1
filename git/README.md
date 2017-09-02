After some work I have created the basis for my “open source” mod.  

Now before I begin I understand that

A) There exists DIY for mods.  Yes, I know.  I am not really interested nor do I really care about the details of those.  I am only really interested in the “intellectual property” and supply chain aspects of how the work I describe can be used to create “dual use” and “open source” on a vast commercial scale.

B) My ideas are dumb.  I accept that and I don’t care what you think in that regard.

C) The mods and technology I describe are not what you like, not “main stream,” etc.  I don’t care.

D) This version is “too big.”  Yes it is.  So is my iPhone 6S.  All I care about is that it serves my purpose.  Change, like winter, will come.

E) Someone’s already published or put out DIY like this.  Again, I don’t care for the reason stated above.

What I am interested is in creating what is effectively “Linux” and “commodity hardware” for vaping.  (If you don’t know what these are you can stop reading now.)

I am currently unaware of anyone attempting this, but if someone is, please let me know.

Everything I do, and hopefully that others contribute, is intended to be fully “open source” and fully sharable.

Everything I post subsequently related to mods and vaping that contain software or hardware designs are published under the MIT Open Source License printed below:

Copyright 2017 Lexigraph, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software, electronic circuit designs and schematics, hardware designs, images and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

The first release I am making under this license is “Open Source Mod #1” (OSM1).

It’s heritage is the PrimusZ and so what’s here is oriented toward manufacturing (no, it’s not manufacturable directly as is but I am keeping that in mind).

I have posted here (http://lwgat.blogspot.com/2017/08/open-source-vaping-overview.html) about my goals.

OSM1 has a couple of things that I think make it interesting: No mechanical buttons, dynamic (while-you-vape) control over power level, mechanical sleep mode.

These may or may not be interesting - I haven’t ever tried the first two.  The third item is so that we can attach wireless radio software and the mod won’t run down while not in use.  It’s not a requirement by any means but it’s interesting.
This version uses an Arduino Micro as a controller (here as sold by Adafruit https://www.adafruit.com/product/1086 and https://store.arduino.cc/usa/arduino-micro).

Why not use a DNA-200 or whatever you ask?  Because I want to tinker with the software and the Atmel 32u4 (http://www.atmel.com/Images/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf) used on it (and on the Adafruit Feather Bluefruit LE https://www.adafruit.com/product/2829) which is essentially an open source PrimusZ controller.

(Arduino and Adafruit publish schematics, PCB layouts, etc. under various open source licenses.)

So to this end below are some pictures of the case, the schematics and software.  I just finished getting it working so I have no idea if its reliable or safe.  Use my ideas at your own risk.

EVERYTHING PROVIDED HERE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

If anyone has any interest I will publish more details on OSM1.  I am moving on to OSM2 which will use the Bluefruit LE instead of the Arduino Micro.  This will enable your cellphone to access data in the mod, control the mod, and manipulate its performance.

(I am planning on putting everything into GitHub - I will publish the link when I get that done.)


