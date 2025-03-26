/***************************************************************************
### Switches.h
***************************************************************************/


#include "Switches.h"
#include "Arduino.h"
//#include "wiring_digital.h"

Switch::Switch()
{
	enabled = false;
	resetEvents();
}

Switch::~Switch()
{
	end();
}

void Switch::begin(uint32_t pin, SwitchType type, uint32_t debounce_time, uint16_t timeOutDouble) {
	button_pin = pin;
	button_type = type;
	debounce = debounce_time;
	
	pinMode(pin, INPUT_PULLUP);
	
	enabled = 1;
	read_enabled = 1;
	long_press_time = 100;
	timeOutDP = timeOutDouble;
	resetEvents();
}

void Switch::setPin(uint32_t pin) {
	button_pin = pin;
	pinMode(pin, INPUT_PULLUP);
}

void Switch::end() {
	enabled = false;
	pinMode(button_pin, INPUT);
}

SwitchEvent Switch::getEvent() {
	SwitchEvent event;

	if (!event_count)
		return SwitchNothing;

	read_enabled = 0;

	event = events[event_rd++];

	if (event_rd == SWITCH_MAX_EVENTS)
		event_rd = 0;

	event_count--;

	read_enabled = 1;

	return event;
}

void Switch::addEvent(SwitchEvent event) {
	if (event_count == SWITCH_MAX_EVENTS)
		return;

	events[event_wr++] = event;
	if (event_wr == SWITCH_MAX_EVENTS)
		event_wr = 0;

	event_count++;
}

void Switch::resetEvents() {
	read_enabled = 0;
	event_count = event_rd = event_wr = 0;
	read_enabled = 1;
	total_pressed_time = 0;
	pressed_time = 0;
	total_released_time = 0;
}



bool Switch::pressedDouble() {
	if(event_dp) {
		event_dp = false;
		return true;
	}
	else {
		return false;
	}	
}

void Switch::resetDouble() {
	pressed_count = 0;
	event_dp = false;
	timeDoublePress = millis();
}


void Switch::poll() {
	uint32_t elapsed = 0;

	if (!enabled)
		return;

	if (read_enabled) {
		// Switch status
		uint8_t status = digitalRead(button_pin);
		
		if ((button_type == LatchingSwitchNO) || (button_type == MomentarySwitch) || (button_type == MomentarySwitchSolo))
			status = !status;
		
		// Stores the raw (un debounced) state for simple reading methods
		raw_state = status;
		
		switch(switchStateMachine) {
			
			case SWITCH_NOT_STABLE:
				if(status != last_read)
					counter = millis();
				else {	
					if (stable_state != status) {
						elapsed = millis() - counter;
						if (elapsed >= debounce) {
							stable_state = status;
							switchStateMachine = SWITCH_STABLE;
							//printf("SWITCH_STABLE\n");
						}
					} // end IF not stable yet
					else {	// stable (debounce wise)
						if (stable_state) {
							// We don't need the long press even => only detect a variable long press after starting pressing
							// We use the total_pressed_time to find out
							total_pressed_time  = millis() - pressed_time;
							//printf("total_press_time: %dms\n", total_pressed_time);
						}
						else {
							total_released_time  = millis() - released_time;
						}
					}					
				}
				break;
				
			case SWITCH_STABLE:
				if (stable_state) {
					long_press_detected = 0;
					total_pressed_time = 0;
					pressed_time = millis();
					switchStateMachine = SWITCH_WAIT_WHILE_PRESSED;
					//printf("SWITCH_WAIT_WHILE_PRESSED\n");
	
				} else {
					total_released_time = 0;
					released_time = millis();
					switchStateMachine = SWITCH_WAIT_WHILE_RELEASED;
					//printf("SWITCH_WAIT_WHILE_RELEASED\n");
				}
				break;
			
			case SWITCH_WAIT_WHILE_PRESSED:
				total_pressed_time  = millis() - pressed_time;
				if(total_pressed_time > timeOutDP) {
					//printf("DP press timeout\n");
					addEvent(SwitchPressed);
					resetDouble();
					switchStateMachine = SWITCH_NOT_STABLE;
				}
				else {
					if(pressed_count) {	// 1 click already engaged
						//printf("DOUBLE CLICK\n");
						resetDouble();
						addEvent(SwitchDoubleClick);
						event_dp = true;
						switchStateMachine = SWITCH_NOT_STABLE;
					}
					else if(status != last_read) {
						//printf("1 more to DP\n");
						//addEvent(SwitchPressed);
						if(isMomentary()) {
							resetDouble();
							pressed_count++;
						}							
						switchStateMachine = SWITCH_NOT_STABLE;
					}					
				}
				break;
				
			case SWITCH_WAIT_WHILE_RELEASED:
				total_released_time  = millis() - released_time;
				if(total_released_time > timeOutDP) {
					addEvent(SwitchReleased);
					resetDouble();
					//printf("DP timeout\n");
					switchStateMachine = SWITCH_NOT_STABLE;
				}
				else if(status != last_read) {
					addEvent(SwitchReleased);
					switchStateMachine = SWITCH_NOT_STABLE;
				}
				break;
			
			default:
				switchStateMachine = SWITCH_NOT_STABLE;
				break;
			
		} // End of SWITCH(switch state machine)
		
		last_read = status;
	} // end of switch (physcical) state reading	
}
