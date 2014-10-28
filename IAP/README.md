<H1>STM32 IAP</H1>

<h3>Summary</h3>
This directory contains In-Application Programming (IAP) applications for STM32 type microcontrollers. Such applications are also called <em>bootstrap loaders</em> or simply <em>boot loaders</em>.
Once programmed to Flash memory, they allow the reprogramming of the <em>domain application</em> via a serial interface.

All IAPs in this repository share the following features:
<ul>
<li>They occupy the lowest 8KiB Flash memory in the controller</li>
<li>They program the domain application to sector 4 (counting starts at 0) of the Flash memory and higher</li>
<li>They use USART1 for serial communication, the parameters are 115200/8/N/1</li>
<li>The serial protocol used is YMODEM with 16bit CRC; 256B and 1024B block sizes are supported</li>
<li>The IAP application is invoked after each reset of the microcontroller</li>
<li>If a domain application is present in Flash memory, the IAP application passes control immediately along to this application after a microcontroller reset. The IAP application can be kept running in two ways (see the readme file of the specific IAP project for details):
<ol>
<li>By pulling a specific input pin to GND during start-up (a user pushes a button)</li>
<li>By writing a specific bit pattern to a specific RAM location and then invoking a microcontroller reset (the domain application does that)</li>
</ol>
<li>Once running the IAP application exits in one of these ways:
<ol>
<li>The microcontroller is reset</li>
<li>After programming a domain application, the IAP passes control along to it</li>
<li>If it receives the characters 'E' or 'e' on the serial line while no YMODEM transfer is in progress, control is passed along to the domain application</li>
</ol>
<li>If no domain application is present in Flash memory, the IAP stays running in any case.</li>
</ul>

<h3>Notes</h3>
<ul>
<li>The Flash sectors between the lowest 8KiB memory area (for the IAP) and the 4th sector (counting starts at 0) can be used for domain application's configuration and EEPROM emulation.</li>
<li>Terminal programs that support the YMODEM protocol: minicom on Linux, TeraTerm on windows</li>
<li>The YMODEM protocol has been chosen because:
<ul>
<li>It supports block transfers of 1KiB size, which is a suitable size for the task</li>
<li>It supports block wise flow control (erase cycles can take several hundred milliseconds)</li>
<li>All transferred blocks contain a 16bit CRC checksum
<li>It transfers the size of the file to be programmed in the first block</li>
</ul>
</ul>

<h3>TODO</h3>
<ul>
<li>Add instructions how to program the IAP</li>
<li>Add hint how to get IAP for similar processors</li>
<li>Future features of IAP: 1st sector of domain application is selectable</li>
<li>Add hint to Nucleo boards</li>
<li>Add links to http://bitcontrol.ch</li>
