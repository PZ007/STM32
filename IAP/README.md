<H1>STM32 IAP</H1>

This repository contains In-Application Programming (IAP) applications for STM32 type microcontrollers. Such applications are also called <em>bootstrap loaders</em> or simply <em>boot loaders</em>.
Once programmed to Flash memory, they allow the reprogramming of the <em>domain application</em> via a serial interface.

All IAPs in this repository share the following specifications:
<ul>
<li>They occupy the lowest 8KiB Flash memory in the controller</li>
<li>They program the domain application to sector 4 (counting starts at 0) of the Flash memory and higher</li>
<li>They use USART1 for serial communication, the parameters are 115200/8/N/1</li>
<li>The serial protocol used is YMODEM with 16bit CRC; 256B and 1024B block sizes are supported</li>
</ul>

Notes:
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

