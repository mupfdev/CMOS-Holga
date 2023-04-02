# CMOS Holga

Some time ago, an old friend reminded me of a project I had completely
forgotten about, in which I made an analogue 35mm camera out of an empty
stove matchbox.  The [resulting photos](media/matchbox-cam-ertesx.jpg)
were nothing to write home about, but the fact that it worked at all
felt good, and the mere mention of it by a friend more than ten years
later proves that it left a lasting impression.

A lot has happened since then, my life has changed, my knowledge and
skills have developed.  Time for a follow-up project!

## What is this all about?

Before I show you the result, a few words about the terms used in the
title.

### CMOS

Complementary metal–oxide–semiconductor (CMOS) technology is used for
constructing integrated circuit (IC) chips, including microprocessors,
microcontrollers, memory chips and other digital logic circuits.  CMOS
technology is also used for analog circuits such as image sensors.

### Holga

The Holga is a medium format 120 film camera, made in Hong Kong, known
for its low-fidelity aesthetic.

The Holga's low-cost construction and simple meniscus lens often yields
pictures that display vignetting, blur, light leaks and other
distortions.  The camera's limitations have brought it a cult following
among some photographers, and Holga photos have won awards and
competitions in art and news photography.

### Mix and match!

As if using a Holga wasn't obscure enough, I decided to bring Nintendo
into the mix.  I dug out my old GameBoy Camera and removed its image
sensor, the Mitsubishi M64282FP.

The [Mitsubishi M64282FP](docs/Mitsubishi_M64282FP.pdf) is a 128×128
pixel CMOS image sensor with built-in image processing and analogue
image output adjustment functions.  It can capture an image and process
it at the same time, just like the human retina.

I used an STM32F401 microcontroller board to read the image data and
store the final images on an SD card via SPI.  This was partly because I
had some in stock and partly because I am familiar with the product
family.  I quickly got some useful results and was motivated to finish
what I'd started.

Since the image sensor needs 5V input voltage, I helped with a step-up
converter.  The camera is powered by a LiPo battery and can be charged
via USB using a TP4056 charging circuit.  The logic levels of the STM32
are sufficient and do not need any level conversion.

### Conclusion

In the end there was not much left of the Holga. The large lens was no
use for the tiny image sensor, so I decided to adapt the optics from the
GameBoy Camera.  It was still a good choice.  The Holga is very cheap
and widely available, and the body is very light with plenty of room for
the electronics.

I could certainly improve the firmware to output JPEGs directly instead
of Portable Grey Map (PGM) files, but overall I am very happy with the
result.  The CMOS Holga is portable, super lightweight and produces
quirky black and white images with a charm of their own.  The camera has
no display, no automatic exposure control and no adjustable focus – it
just offers the raw basics of a simple point-and-shoot camera.  No
extras, no gimmicks – just what I had in mind.

## Gallery

Click on the images to view them in full resolution.

### Development stage

[![](media/prototype-tn.jpg)](media/prototype.jpg?raw=true "Prototype")
[![](media/stm32f4-step-up-converter-tn.jpg)](media/stm32f4-step-up-converter.jpg?raw=true "STM32F4 with step-up converter")
[![](media/build-1-tn.jpg)](media/build-1.jpg?raw=true "Build process 1")
[![](media/build-2-tn.jpg)](media/build-2.jpg?raw=true "Build process 2")
[![](media/optics-1-tn.jpg)](media/optics-1.jpg?raw=true "Optics adaption 1")
[![](media/optics-2-tn.jpg)](media/optics-2.jpg?raw=true "Optics adaption 2")
[![](media/installation-tn.jpg)](media/installation.jpg?raw=true "Installation process")
[![](media/front-view-tn.jpg)](media/front-view.jpg?raw=true "Front view")

### Shots

More to follow.

[![selfie](media/selfie.jpg)](media/selfie.jpg?raw=true "selfie")
