# binary-keyboard
## An STM32F0 based binary keyboard
Hi, so I've decided to make a keyboard with just 2 buttons on in to fool the new guys at my office, so me and my boss went head first to this project. It uses a ST microcontroller with native USB support and the exterior (excluding the keys) was 3D printed.

![bin keyboard](https://raw.githubusercontent.com/predebon/binary-keyboard/master/bin_kbrd.png)

### Hardware and Microcontroller
I'm using my STM32F042F6 capibara 2 mini dev kit, having 2 buttons, 3 LEDs and a switch for the bootloader, on top of that, i've soldered 3 buttons to a perf board with the same size of my board, making it the 0, 1 and backspace buttons. The pull ups are internal.

The pinout is as follows:
>PA1 - Button '0'

>PA2 - Button '1'

>PA3 - Button 'Backspace'

>PB8 - Used to enter on DFU mode / when booted, used to enter in debug mode

On this project, the LED resistors were removed to avoid blinking lights inside of the case.

### Using the Keyboard
It is plug and play, just make sure your keyboard is configured to standard US layout.
When you type groupings of 8 bits, the keyboard interprets it as a byte and writes the respective ASCII value.

### How it Works
When you type, the buffer is filled MSB first, so that you can read easily chars from the ASCII table. When the buffer is full, it is converted to the HID Report equivalent to that ASCII 

Feel free to use this code for all sorts of STM32F0 applications, the function _easterEgg()_ can be used to write automatically any phrase you want.




