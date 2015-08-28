This code represents a porting of VCP functionality to the STM32F4-Discovery
board from the STM3240G example in the USB Host v2.1.0 firmware library.
Original Copyrights have been left as this is mostly a derivative work.

The USART is expected to be wired as follows

TX PA2
RX PA3

Connect these to a 3V CMOS compatible serial device, along with GND, and VDD
as required.

The USART is configured as 115200 baud 8N1, data from the USB VCP and physical
USART exchange.

On the PC side you will need the ST VCP v1.3.1 driver installed.

http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/SW_DRIVER/stm32_vcp.zip
http://dekar.wc3edit.net/st%20website/VCP_Setup.zip

If the VCP does not connect properly unplug the USB, and then plug in again.
Please refer to the original source and documentation of the USB Host library
for more insight. I make no representations to the function or bug free
operation of this code.

This code builds in Windows using the Keil uVision 4.xx tools

Install the STM32F4-Discovery firmware library under
  \Keil\ARM\Examples\ST or your project folder structure

Unpack this project under
  \Keil\ARM\Examples\ST\STM32F4-Discovery_FW_V1.1.0\Project

So you have
  \Keil\ARM\Examples\ST\STM32F4-Discovery_FW_V1.1.0\Project\VCP

Build the project, download, and debug.

If you have questions please try to formulate them into a coherent thought,
assuming I know nothing about you, your board, your development environment,
your project or the tests you may have already done. Title your email as a
summary of the problem, and not something retarded like "HELP!!". Emails
aren't character limited, describe things fully, and don't use TXT
abbreviations.

Donations of equipment, boards, and money to further development would be
welcome. Consider how much of your time has been saved by the open
availability of this package.

clive - sourcer32@gmail.com
