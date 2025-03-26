/***************************************************************************
### Switches.h
***************************************************************************/

#ifndef __SWITCHES_H__
#define __SWITCHES_H__


#define SWITCH_MAX_EVENTS	5

#include "Arduino.h"

typedef enum
{	LatchingSwitchNC = 0,
	LatchingSwitchNO,
	MomentarySwitch,
	MomentarySwitchSolo
} SwitchType;


typedef enum
{
	SwitchNothing = 0,
	SwitchPressed,
	SwitchReleased,
	SwitchLongPressed,
	SwitchShortPressAndRelease,
	SwitchDoubleClick
} SwitchEvent;


enum s_switchStateMachine {
	SWITCH_NOT_STABLE = 0,
	SWITCH_STABLE,
	SWITCH_WAIT_WHILE_PRESSED,
	SWITCH_WAIT_WHILE_RELEASED,
};


class Switch
{
public:
	Switch();
	~Switch();

	void begin(uint32_t pin, SwitchType type = MomentarySwitch, uint32_t debounce_time = 25, uint16_t timeOutDouble = 0);
	void setPin(uint32_t pin);
	uint32_t getPin() { return button_pin; }
	void end();
	inline bool pressed() { return stable_state == 1; }
	bool pressedDouble();
	void setDebounce(int debounce_time) { debounce = debounce_time; }
	
	void resetDouble();
	inline bool pressedRaw() { return raw_state == 1; }
	inline bool released() { return stable_state != 1; }
	inline bool releasedRaw() { return raw_state == 1; }
	inline void setLongPressTime(uint32_t ms) { long_press_time = ms; }
	inline void setTimeOutDouble(uint32_t ms) { timeOutDP = ms; }
	inline uint32_t getTimeOutDouble() { return timeOutDP; }
	inline uint32_t getDebounce() { return debounce; }
	uint32_t getPressTime(void) { return total_pressed_time; }
	uint32_t getReleaseTime(void) { return total_released_time; }
	
	inline bool isMomentary() {return (button_type == MomentarySwitch) || (button_type == MomentarySwitchSolo); }
	inline bool isLatching() {return (button_type != MomentarySwitch) && (button_type != MomentarySwitchSolo); }
	inline bool isSolo() {return (button_type == MomentarySwitchSolo); }
	SwitchEvent getEvent();
	void resetEvents();
	void poll();

protected:
	void addEvent(SwitchEvent event);

private:
	volatile SwitchEvent events[SWITCH_MAX_EVENTS];
	uint8_t event_count;
	uint8_t event_rd;
	uint8_t event_wr;
	
	volatile uint8_t switchStateMachine;

	volatile uint8_t read_enabled;
	uint8_t enabled;

	volatile uint8_t stable_state;
	volatile uint8_t raw_state;
	
	uint32_t debounce;
	uint32_t pressed_time;
	uint32_t released_time;
	uint32_t total_pressed_time;
	uint32_t total_released_time;
	uint32_t long_press_time;
	volatile uint8_t last_read;
	uint8_t long_press_detected;
	volatile uint32_t counter;
	SwitchType button_type;
	uint32_t button_pin;
	
	volatile uint8_t pressed_count;
	volatile uint8_t statusDouble;
	volatile uint8_t already;
	volatile uint32_t timeDoublePress;
	uint16_t timeOutDP;
	volatile uint8_t event_dp;
};

#endif // __SWITCHES_H__
