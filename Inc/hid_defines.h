#define F1 0x3a
#define F2 0x3b
#define F3 0x3c
#define F4 0x3d
#define F5 0x3e
#define F6 0x3f
#define F7 0x40
#define F8 0x41
#define F9 0x42
#define F10 0x43
#define F11 0x44
#define F12 0x45

// keys
#define ENTER 0x28
#define ESCAPE 0x29
#define BACKSPACE 0x2a
#define TAB 0x2b
#define SPACEBAR 0x2c
#define CAPSLOCK 0x39
#define PRINTSCREEN 0x46
#define SCROLLLOCK 0x47
#define PAUSE 0x48
#define INSERT 0x49
#define HOME 0x4a
#define PAGEUP 0x4b
#define DELETE 0x4c
#define END 0x4d
#define PAGEDOWN 0x4e
#define RIGHTARROW 0x4f
#define LEFTARROW 0x50
#define DOWNARROW 0x51
#define UPARROW 0x52

//modifiers
#define CTRL 0x01
#define SHIFT 0x02
#define ALT 0x04
#define GUI 0x08

const static uint8_t HIDTable[] =  {
	0x00, // 0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x28, // 10
	0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 20
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, // 30
	0x00, 0x2c, 0x1e, 0x34, 0x20, 0x21, 0x22, 0x24, 0x34, 0x26, // 40
	0x27, 0x25, 0x2e, 0x36, 0x2d, 0x37, 0x38, 0x27, 0x1e, 0x1f, // 50
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x33, 0x33, 0x36, // 60
	0x2e, 0x37, 0x38, 0x1f, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, // 70
	0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, // 80
	0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, // 90
	0x2f, 0x31, 0x30, 0x23, 0x2d, 0x35, 0x04, 0x05, 0x06, 0x07, // 100
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, // 110
	0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, // 120
	0x1c, 0x1d, 0x2f, 0x31, 0x30, 0x35, 127 // 127
};

const static uint8_t modifierTable[] = {
	0x00, // 0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 10
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 20
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 30
	0x00, 0x00, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, 			// 40
	SHIFT, 0x00, SHIFT, SHIFT, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 50
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, SHIFT, 0x00, SHIFT, 					// 60
	0x00, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, 			// 70
	SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT,			// 80
	SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, SHIFT, 			// 90
	0x00, 0x00, 0x00, SHIFT, SHIFT, 0x00, 0x00, 0x00, 0x00, 0x00, 						// 100
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 110
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 					// 120
	0x00, 0x00, SHIFT, SHIFT, SHIFT, SHIFT, 0x00 // 127
};
