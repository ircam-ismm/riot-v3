/***************************************************************************

### colors.cpp

***************************************************************************/

#include "colors.h"





CRGBW8 alphaBlend(CRGBW8 &top, CRGBW8 &bottom, float alpha_top) {
	uint32_t r, g, b, w;

	if (alpha_top > 1)
		alpha_top = 1;
	else if (alpha_top < 0)
		alpha_top = 0;
	
	float alpha_bottom = 1.0 - alpha_top;

	r = (uint32_t) (getRed(top) * alpha_top) + (uint32_t) (getRed(bottom) * alpha_bottom);
	g = (uint32_t) (getGreen(top) * alpha_top) + (uint32_t) (getGreen(bottom) * alpha_bottom);
	b = (uint32_t) (getBlue(top) * alpha_top) + (uint32_t) (getBlue(bottom) * alpha_bottom);
	w = (uint32_t) (getWhite(top) * alpha_top) + (uint32_t) (getWhite(bottom) * alpha_bottom);

	return RGBW8(r,g,b,w);
}


CRGBW8 alphaBlend(CRGBW8 &top, CRGBW8 &bottom, int alpha_top) {
	uint32_t r, g, b, w;

	alpha_top = constrain(alpha_top, 0, 255);
	int alpha_bottom = 255 - alpha_top;
	
	r = (alpha_top * top[RED] + alpha_bottom * bottom[RED]) >> 8;
	g = (alpha_top * top[GREEN] + alpha_bottom * bottom[GREEN]) >> 8;
	b = (alpha_top * top[BLUE] + alpha_bottom * bottom[BLUE]) >> 8;
	w = (alpha_top * top[WHITE] + alpha_bottom * bottom[WHITE]) >> 8;

	return RGBW8(r,g,b,w);
}



// Full random color
CRGBW8 randomColor(bool normalize) {
	
	int array[4];
	int peak = 0;
	for(int i = 0 ; i < 4 ; i++) {
		array[i] = getRandom(0,255);
		if(array[i] > peak)
			peak = array[i];
	}
	
	if(normalize) {
		int mult = 255 * peak;
		for(int i = 0 ; i < 3 ; i++) { // RGB only not using the white channel in normalization
			array[i] = (array[i] * 255) / peak;
			array[i] = constrain(array[i], 0, 255);
		}

	}
	return CRGBW8(array[RED], array[GREEN], array[BLUE], array[WHITE]);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

CRGBW8::CRGBW8() :
	r(0), g(0), b(0), w(0)
{
}

CRGBW8::~CRGBW8() {

}

CRGBW8::CRGBW8(uint32_t color) :
	w(color & 0xFF),
	b((color >> 8) & 0xFF),
	g((color >> 16) & 0xFF),
	r((color >> 24) & 0xFF)
{
}

CRGBW8::CRGBW8(uint8_t r, uint8_t g, uint8_t b, uint8_t w) :
	r(r), g(g), b(b), w(w)
{
}



void CRGBW8::print(void) {
	Serial.printf("[R]:%d - [G]:%d - [B]:%d - [W]:%d\n", r, g, b, w);
}

CRGBW8 CRGBW8::operator= (const uint32_t color)
{
	w = (color & 0xFF);
	b = ((color >> 8) & 0xFF);
	g = ((color >> 16) & 0xFF);
	r = ((color >> 24) & 0xFF);
	return *this;
}



CRGBW8 CRGBW8::operator* (float val) {
	if (val > 1)
		val = 1;
	else if (val < 0)
		val = 0;

	return CRGBW8(r * val, g * val, b * val, w * val);
}

CRGBW8 CRGBW8::operator* (int val) {
	val = constrain(val, 0, 255);
	if(val == 255)
		return(*this);
	else
		return CRGBW8((r * val) >>8, (g * val) >>8, (b * val) >>8, (w * val) >>8);
}


inline void CRGBW8::operator*= (float value) {
	*this = *this * value;
}

inline void CRGBW8::operator*= (int value) {
	*this = *this * value;
}

inline bool CRGBW8::operator== (const CRGBW8& color) {
	return (color.r == r && color.g == g && color.b == b && color.w == w);
}

inline bool CRGBW8::operator!= (const CRGBW8& color) {
	return (color.r != r || color.g != g || color.b != b || color.w != w);
}

inline bool CRGBW8::operator!= (const uint32_t color) {
	CRGBW8 match(color);

	if (match.r != r || match.g != g || match.b != b || match.w != w)
		return true;
	return false;
}

// Accessor
uint8_t CRGBW8::operator[] (int channel) const
{
	switch(channel) {
		case RED:
			return(r);
			break;
		
		case GREEN:
			return(g);
			break;	
			
		case BLUE:
			return(b);
			break;	
		
		case WHITE:
			return(w);
			break;
		
		default:
			return(0);
			break;
	}
}

// Mutator
uint8_t& CRGBW8::operator[] (int channel) {
	switch(channel) {
		case RED:
			return(r);
			break;
		
		case GREEN:
			return(g);
			break;	
			
		case BLUE:
			return(b);
			break;	
		
		case WHITE:
			return(w);
			break;
		
		default:
			return(r);
			break;
	}
}


inline bool CRGBW8::operator! () const {
	return !notBlack();
}

inline bool CRGBW8::notBlack() const {
	return (r || g || b || w);
}


bool CRGBW8::isBlack() const {
	return (!r && !g && !b && !w);
}

bool CRGBW8::isGrey() const {
	return ((r == g) && (g == b));
}

bool CRGBW8::isWhite() const {
	return (isGrey() && (r == 255));
}


inline uint32_t CRGBW8::toInt() const {
	return (r | ((uint32_t) g << 8) | ((uint32_t) b << 16) | ((uint32_t) w << 24));
}

uint32_t CRGBW8::toIntWRGB() const {
    return (((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}


void CRGBW8::Black() {
	r = g = b = w = 0;
	//r = g = b = w = 2;
}


// Missing operator + and +=
CRGBW8 CRGBW8::operator+ (const CRGBW8 &other) {
	uint32_t rr, gg, bb, ww;
	rr = r + other.r;
	rr = constrain(rr, 0, 255);
	gg = g + other.g;
	gg = constrain(gg, 0, 255);
	bb = b + other.b;
	bb = constrain(bb, 0, 255);
	ww = w + other.w;
	ww = constrain(ww, 0, 255);

	return CRGBW8((uint8_t)rr, (uint8_t)gg, (uint8_t)bb, (uint8_t)ww);
}

inline void CRGBW8::operator+= (const CRGBW8 &other) {
	*this = *this + other;
}


CRGBW8 CRGBW8::blend(CRGBW8& top, CRGBW8& bottom, float alpha_top) {
	uint32_t r, g, b, w;

	if (alpha_top > 1)
		alpha_top = 1;
	else if (alpha_top < 0)
		alpha_top = 0;

	float alpha_bottom = 1. - alpha_top;

	r = (uint32_t)((float)top.r * alpha_top) + (uint32_t)((float)bottom.r * alpha_bottom);
	g = (uint32_t)((float)top.g * alpha_top) + (uint32_t)((float)bottom.g * alpha_bottom);
	b = (uint32_t)((float)top.b * alpha_top) + (uint32_t)((float)bottom.b * alpha_bottom);
	w = (uint32_t)((float)top.w * alpha_top) + (uint32_t)((float)bottom.w * alpha_bottom);

	
	return CRGBW8(r,g,b,w);
}

void CRGBW8::blend(const CRGBW8& other, float amount) {
	if (amount > 1.)
		amount = 1.;
	else if (amount < 0.)
		amount = 0.;
	
	float alpha_bottom = 1. - amount;	// We do the calculation once
	
	r = (uint8_t)(((float)other.r * amount) + ((float)r * alpha_bottom));
	g = (uint8_t)(((float)other.g * amount) + ((float)g * alpha_bottom));
	b = (uint8_t)(((float)other.b * amount) + ((float)b * alpha_bottom));
	w = (uint8_t)(((float)other.w * amount) + ((float)w * alpha_bottom));
	
}

void CRGBW8::blend(const CRGBW8 &other, int x) {

	if(x == 0)
		return;
	
	x = constrain(x, 0, 255);
	int y = 255 - x;
	
	r = (x * other[RED] + y * r) >> 8;
	g = (x * other[GREEN] + y * g) >> 8;
	b = (x * other[BLUE] + y * b) >> 8;
	w = (x * other[WHITE] + y * w) >> 8;
}

CRGBW8 CRGBW8::blendThis(const CRGBW8 &other, int x) {


	x = constrain(x, 0, 255);
	int y = 255 - x;
	
	r = (x * other[RED] + y * r) >> 8;
	g = (x * other[GREEN] + y * g) >> 8;
	b = (x * other[BLUE] + y * b) >> 8;
	w = (x * other[WHITE] + y * w) >> 8;
	return(*this);
}

void CRGBW8::blendPercent(const CRGBW8& other, int amount) {
	if (amount > 100)
		amount = 100;
	
	float val = amount / 100;
	//Serial.printf("blend int\n");

	blend(other, val);
}


void  CRGBW8::randColor() {
	r = random(255);
	g = random(255);
	b = random(255);
	w = random(255);
}

void  CRGBW8::randomize(int amount) {
	r = r - (random(amount) * r / 100);
	g = g - (random(amount) * g / 100);
	b = b - (random(amount) * b / 100);
	w = w - (random(amount) * w / 100);
}


void CRGBW8::randomColor(bool normalize) {
	
	int array[4];
	int peak = 0;
	for(int i = 0 ; i < 4 ; i++) {
		array[i] = getRandom(0,255);
		if(array[i] > peak)
			peak = array[i];
	}
	
	if(normalize) {
		int mult = 255 * peak;
		for(int i = 0 ; i < 3 ; i++) { // RGB only not using the white channel in normalization
			array[i] = (array[i] * 255) / peak;
			array[i] = constrain(array[i], 0, 255);
		}

	}
	
	r = array[RED];
	g = array[GREEN]; 
	b = array[BLUE];
	w = array[WHITE];
}


void CRGBW8::GRB() {
	uint8_t temp;
	temp = r;
	r = g;
	g = temp;
}





// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
CRGBW8 wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return CRGBW8(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return CRGBW8(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  } 
  else {
    WheelPos -= 170;
    return CRGBW8(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}


CRGBW8 wheelHue(hsv& HSV) {
	return wheelHue(HSV.h, HSV.s, HSV.v);
}

// Input a true HUE selection over a palette range
CRGBW8 wheelHue(int hue, int sat, int brightness) {
	// Remaps using the perceptive CIE (human) lookup table of the brightness and saturation
	brightness = cie_lut[brightness];
	sat = 255-cie_lut[255-sat];
	
	int r, g, b, base;
	
	if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
		return CRGBW8(brightness, brightness, brightness, 0);  
	}
	else if (hue == 360)
		return White;
	else { 

		base = ((255 - sat) * brightness)>>8;

		switch(hue/60) {
			case 0:
				r = brightness;
				g = (((brightness-base)*hue)/60)+base;
				b = base;
			break;

			case 1:
				r = (((brightness-base)*(60-(hue%60)))/60)+base;
				g = brightness;
				b = base;
			break;

			case 2:
				r = base;
				g = brightness;
				b = (((brightness-base)*(hue%60))/60)+base;
			break;

			case 3:
				r = base;
				g = (((brightness-base)*(60-(hue%60)))/60)+base;
				b = brightness;
			break;

			case 4:
				r = (((brightness-base)*(hue%60))/60)+base;
				g = base;
				b = brightness;
			break;

			case 5:
				r = brightness;
				g = base;
				b = (((brightness-base)*(60-(hue%60)))/60)+base;
			break;
		}
	}
	return CRGBW8(r, g, b, 0);  
}



enum s_Order {
	STRING_BEFORE = 0, 
	STRING_AFTER = 1,
	STRING_EQUAL = 2
};


uint8_t beforeOrAfter(char *compareStr, char *refStr) {
	
	if(!strcmp(compareStr, refStr))
		return STRING_EQUAL;
	
	char str1[30], str2[30];
	int refSize = strlen(refStr);
	int compareSize = strlen(compareStr);
	int useSize = (refSize >= compareSize) ? refSize : compareSize;
	
	strncpy(str1, compareStr, useSize);
	strncpy(str2, refStr, useSize);
	//strlwr(str1);
	//strlwr(str2);
	int i;
	for(i = 0 ; i < useSize ; i++) {
		if(str1[i] != str2[i])
			break;
	}
	if(str1[i] < str2[i])
		return STRING_BEFORE;
	else
		return STRING_AFTER;
}


// Colors are organized in alphabetic order. Use dichotomy to quickly find the right name and associated color.
CRGBW8 getColorFromDictionary(char *name) {
	CRGBW8 color(Black);
	int howmanyColors = COLOR_DICTIONARY_ELEMENTS;
	int searchRange = (howmanyColors-1) / 2;
	int index = searchRange;
	//Serial.printf("dictionnary has %d items\n", howmanyColors);
	//Serial.printf("Looking for string [%s]\n", name);
	char lowerName[30];
	char lowerDict[30];

	bool quit = false;
	int timeout = (howmanyColors / 2) + 10;	// Searching should not be longer than this to find the name.
	int iterations = 0;
	
	strcpy(lowerName, name);
	strlwr(lowerName);
	
	while(!quit && timeout) {
		//Serial.printf("index=%d\n", index);
		strcpy(lowerDict, ColorDictionary[index].name);
		strlwr(lowerDict);

		if(!strcmp(lowerName, lowerDict)) {
			quit = true;
			color = CRGBW8(ColorDictionary[index].color);
			//Serial.printf("Found name [%s] matching with [%s] in dictionary index [%d] after  %d iterations\n", name, ColorDictionary[index].name, index, iterations);
			//Serial.printf("Color = 0x%08X\n", ColorDictionary[index].color);
			break;
		}
			
		if(beforeOrAfter(lowerName, lowerDict) == STRING_BEFORE) {
			//Serial.printf("lower\n");
			searchRange = searchRange / 2;
			searchRange = constrain(searchRange, 1, (howmanyColors-1) / 2);
			index = index - searchRange;
			index = constrain(index, 0, howmanyColors-1);
		}
		else {
			//index = index + (((howmanyColors-1) - index) / 2);
			//Serial.printf("upper\n");
			searchRange = searchRange / 2;
			searchRange = constrain(searchRange, 1, (howmanyColors-1) / 2);
			index = index + searchRange;
			index = constrain(index, 0, howmanyColors-1);
		}
		iterations++;
		timeout--;
		
	} // end of search
	if(!timeout)
		printf("Color Name [%s] not found in dictionary->Black\n", name);
	//else
	//	Serial.printf("remaining search attempts = %d\n", timeout);
	
	return color;
}


