<H1>STM32F401 IAP</H1>

<h3>Summary</h3>
This directory contains the In-Application Programming (IAP) application for the STM32F401 microcontroller.

<h3>Application Features</h3>
See the IAP top-level <a href="https://github.com/bitcontrol/STM32/blob/master/IAP/README.md" title="IAP top-level README.md" target="_blank">README.md</a> file for the common features of this application. Specific to it is:
<ul>
<li>If a domain application is present in Flash memory, the IAP application passes control immediately along to this application after a microcontroller reset. The IAP application can be kept running in two ways:
<ol>
<li>By pulling the pin PC4 to GND during start-up (a user pushes a button)</li>
<li>By writing the bit pattern 0x12345678 in little endian format to the RAM location 0x20000000 (that's the SRAM base address) and then invoking a microcontroller reset; the domain application has to be programmed accordingly</li>
</ol>
</ul>

<h3>Program IAP Application to Target</h3>
This directory contains a file called <em>IAP_STM32F401.hex</em>. That's the binary image of the IAP application in Intel's HEX binary format. Use the tools of your choice to program it to the bottom end of the microcontroller's Flash memory area (address 0x8000000). It consumes less than 8KiB and therefore fits into sector 0.
The programming has successfully been tested using
<ul>
<li>A STMicroelectronics Nucleo board of type NUCLEO-F401RE</li>
<li>The ST-LINK connection between the host and the target board; this requires a USB cable with the plugs <em>Standard A to Mini B</em></li>
<li>The Keil IDE uVision version V5.10.0.2</li>
</ul>
The uVision project file as well as its option file are part of this directory. Follow the instructions <a href="http://www.keil.com/support/man/docs/uv4/uv4_fl_hexdownload.htm" target="_blank">here</a> if you need help.

<h3>Notes</h3>
<ul>
<li>This application might also work for other STM32F4xx devices. If it doesn't for yours, send an e-mail to info@bitcontrol.ch.</li>
</ul>
