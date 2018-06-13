# RaspberryPi-NHD-2.4-TFT
C executable for the RaspberryPi to output an area of the framebuffer to the NHD-2.4-240320CF TFT (ST7789, 16 bit parallel)

This is a driver for a Newhaven 2.4" TFT display (http://www.newhavendisplay.com/tfts-24-tfts-c-1_590.html). It maps the framebuffer of the main display to memory, extracts a 320x240 pixel sized area and sends this to the TFT display via the 16 bit parallel interface.

![TFT showing 320x240 area](/images/raspberry-desktop-320x240.jpg)

*A 320x240 area of the Raspberry Pi's desktop rendered on the TFT*

The driver init code sets up the 16-bit parallel interface to the color mode 65K RGB with 5/6/5 bits for the red/green/blue pixel values.

The pixel data in the framebuffer is 32 bits per pixel, 8/8/8/8 bits for the alpha/red/green/blue pixel values. This 32 bit data is converted to the 5/6/5 16 bit format and send to the TFT driver IC via the GPIOs with a frame rate of around 20 fps.

## Physical setup

**Part list:**

* Raspberry Pi 2 or 3
* NHD-2.4-240320CF-CSXN-F TFT display with ST7789S driver IC
* NHD-FFC40 Breakout-Board (40 pin, 0.5mm pitch)


### Connecting the NHD TFT to the Raspberry Pi

Connect the **TFT Pins** from the FFC40 breakout to the **Raspberry's GPIO** (I used simple jumper wires) like this:  
```
          TFT   Raspberry  
    ---------   ---------   
       GND  1   GND  
        NC  2   -  
        NC  3   -    
        NC  4   -  
        NC  5   -   
        NC  6   -  
       VDD  7   3.3v   
     IOVDD  8   3.3v (typ. 1.8V, max 3.3V)   
        NC  9   -   
       /CS 10   GND (= CS is always enabled)   
       D/C 11   PIN 07 / GPIO04   
       /WR 12   PIN 12 / GPIO18   
       /RD 13   PIN 01 / 3.3v (the read function is not used)   
       DB0 14   PIN 13 / GPIO27   
       DB1 15   PIN 15 / GPIO22    
       DB2 16   PIN 16 / GPIO23    
       DB3 17   PIN 18 / GPIO24    
       DB4 18   PIN 22 / GPIO25  
       DB5 19   PIN 24 / GPIO08    
       DB6 20   PIN 26 / GPIO07  
       DB7 21   PIN 29 / GPIO05    
       DB8 22   PIN 31 / GPIO06    
       DB9 23   PIN 32 / GPIO12    
      DB10 24   PIN 33 / GPIO13    
      DB11 25   PIN 35 / GPIO19    
      DB12 26   PIN 36 / GPIO16    
      DB13 27   PIN 37 / GPIO26    
      DB14 28   PIN 38 / GPIO20    
      DB15 29   PIN 40 / GPIO21    
      /RES 30   PIN 11 / GPIO17   
       IM0 31   GND (= selects the 16 bit interface)   
        NC 32   -   
       GND 33   GND   
    LED-K1 34   GND   
    LED-K2 35   GND   
    LED-K3 36   GND   
    LED-K4 37   GND   
     LED-A 38   3.3v    
       GND 39   GND   
        NC 40   -    
```

## Compiling and running the executable

* Download the file **NHD_24_240320_Framebuffer2GPIO.c** from this repository to your Raspberry Pi's home folder
* Open a terminal on the Raspberry Pi and enter this line:  
`gcc -o NHD_24 NHD_24_240320_Framebuffer2GPIO.c -lrt -O3`  
This will generate the executable **NHD_24** in the same folder as the .c file
* To run the executable, enter  
`sudo ./NHD_24`  
into the terminal (or bouble click on the file)
* `CTRL-C` terminates the executable when run from the terminal

## Offsetting the TFT display area

The 320x240 pixel area that is output to the TFT has its origin by default at the upper left corner (0/0) of the display. If you want to move the area to some other location within the display's resolution, you can change the values of the #defines `X_START` and `Y_START` in the NHD_24_240320_Framebuffer2GPIO.c file at line numbers 24 and 25.

