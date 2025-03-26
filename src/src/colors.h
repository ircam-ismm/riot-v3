/***************************************************************************
### colors.h

***************************************************************************/

#ifndef __COLORS_H__
#define __COLORS_H__


#include "Arduino.h"


extern uint32_t getRandom(uint32_t min, uint32_t max);

// remplacement par ENUM class
enum color_order{
	RED = 0,
	GREEN,
	BLUE,
	WHITE
};


#define MAX_10BITS_RANGE			1023


typedef struct s_HSV {
	int h,s,v;
}hsv;
	


// Predefined RGB colors
// We could Use a hash table that is of a size of a power of 2.
// We have about 150 colors to store, hence a 256 hash table size (?)
typedef enum {
	AliceBlue=0xF0F8FF00,
	Amethyst=0x9966CC00,
	AntiqueWhite=0xFAEBD700,
	Aqua=0x00FFFF00,
	Aquamarine=0x7FFFD400,
	Azure=0xF0FFFF00,
	Beige=0xF5F5DC00,
	Bisque=0xFFE4C400,
	Black=0x00000000,
	BlanchedAlmond=0xFFEBCD00,
	Blue=0x0000FF00,
	BlueViolet=0x8A2BE200,
	Brown=0xA52A2A00,
	BurlyWood=0xDEB88700,
	CadetBlue=0x5F9EA000,
	Chartreuse=0x7FFF0000,
	Chocolate=0xD2691E00,
	Coral=0xFF7F5000,
	CornflowerBlue=0x6495ED00,
	Cornsilk=0xFFF8DC00,
	Crimson=0xDC143C00,
	Cyan=0x00FFFF00,
	DarkBlue=0x00008B00,
	DarkCyan=0x008B8B00,
	DarkGoldenrod=0xB8860B00,
	DarkGray=0xA9A9A900,
	DarkGrey=0xA9A9A900,
	DarkGreen=0x00640000,
	DarkKhaki=0xBDB76B00,
	DarkMagenta=0x8B008B00,
	DarkOliveGreen=0x556B2F00,
	DarkOrange=0xFF8C0000,
	DarkOrchid=0x9932CC00,
	DarkRed=0x8B000000,
	DarkSalmon=0xE9967A00,
	DarkSeaGreen=0x8FBC8F00,
	DarkSlateBlue=0x483D8B00,
	DarkSlateGray=0x2F4F4F00,
	DarkSlateGrey=0x2F4F4F00,
	DarkTurquoise=0x00CED100,
	DarkViolet=0x9400D300,
	DeepPink=0xFF149300,
	DeepSkyBlue=0x00BFFF00,
	DimGray=0x69696900,
	DimGrey=0x69696900,
	DodgerBlue=0x1E90FF00,
	FireBrick=0xB2222200,
	FloralWhite=0xFFFAF000,
	ForestGreen=0x228B2200,
	Fuchsia=0xFF00FF00,
	Gainsboro=0xDCDCDC00,
	GhostWhite=0xF8F8FF00,
	Gold=0xFFD70000,
	Goldenrod=0xDAA52000,
	Gray=0x80808000,
	Grey=0x80808000,
	Green=0x00800000,
	GreenYellow=0xADFF2F00,
	Honeydew=0xF0FFF000,
	HotPink=0xFF69B400,
	IndianRed=0xCD5C5C00,
	Indigo=0x4B008200,
	Ivory=0xFFFFF000,
	Khaki=0xF0E68C00,
	Lavender=0xE6E6FA00,
	LavenderBlush=0xFFF0F500,
	LawnGreen=0x7CFC0000,
	LemonChiffon=0xFFFACD00,
	LightBlue=0xADD8E600,
	LightCoral=0xF0808000,
	LightCyan=0xE0FFFF00,
	LightGoldenrodYellow=0xFAFAD200,
	LightGreen=0x90EE9000,
	LightGrey=0xD3D3D300,
	LightPink=0xFFB6C100,
	LightSalmon=0xFFA07A00,
	LightSeaGreen=0x20B2AA00,
	LightSkyBlue=0x87CEFA00,
	LightSlateGray=0x77889900,
	LightSlateGrey=0x77889900,
	LightSteelBlue=0xB0C4DE00,
	LightYellow=0xFFFFE000,
	Lime=0x00FF0000,
	LimeGreen=0x32CD3200,
	Linen=0xFAF0E600,
	Magenta=0xFF00FF00,
	Maroon=0x80000000,
	MediumAquamarine=0x66CDAA00,
	MediumBlue=0x0000CD00,
	MediumOrchid=0xBA55D300,
	MediumPurple=0x9370DB00,
	MediumSeaGreen=0x3CB37100,
	MediumSlateBlue=0x7B68EE00,
	MediumSpringGreen=0x00FA9A00,
	MediumTurquoise=0x48D1CC00,
	MediumVioletRed=0xC7158500,
	MidnightBlue=0x19197000,
	MintCream=0xF5FFFA00,
	MistyRose=0xFFE4E100,
	Moccasin=0xFFE4B500,
	NavajoWhite=0xFFDEAD00,
	Navy=0x00008000,
	OldLace=0xFDF5E600,
	Olive=0x80800000,
	OliveDrab=0x6B8E2300,
	Orange=0xFFA50000,
	OrangeRed=0xFF450000,
	Orchid=0xDA70D600,
	PaleGoldenrod=0xEEE8AA00,
	PaleGreen=0x98FB9800,
	PaleTurquoise=0xAFEEEE00,
	PaleVioletRed=0xDB709300,
	PapayaWhip=0xFFEFD500,
	PeachPuff=0xFFDAB900,
	Peru=0xCD853F00,
	Pink=0xFFC0CB00,
	Plaid=0xCC553300,
	Plum=0xDDA0DD00,
	PowderBlue=0xB0E0E600,
	Purple=0x80008000,
	Red=0xFF000000,
	RosyBrown=0xBC8F8F00,
	RoyalBlue=0x4169E100,
	SaddleBrown=0x8B451300,
	Salmon=0xFA807200,
	SandyBrown=0xF4A46000,
	SeaGreen=0x2E8B5700,
	Seashell=0xFFF5EE00,
	Sienna=0xA0522D00,
	Silver=0xC0C0C000,
	SkyBlue=0x87CEEB00,
	SlateBlue=0x6A5ACD00,
	SlateGray=0x70809000,
	SlateGrey=0x70809000,
	Snow=0xFFFAFA00,
	SpringGreen=0x00FF7F00,
	SteelBlue=0x4682B400,
	Tan=0xD2B48C00,
	Teal=0x00808000,
	Thistle=0xD8BFD800,
	Tomato=0xFF634700,
	Turquoise=0x40E0D000,
	Violet=0xEE82EE00,
	Wheat=0xF5DEB300,
	White=0xFFFFFF00,
	WhiteSmoke=0xF5F5F500,
	Yellow=0xFFFF0000,
	YellowGreen=0x9ACD3200,
	// LED RGB color that roughly approximates
	// the color of incandescent fairy lights,
	// assuming that you're using FastLED
	// color correction on your LEDs (recommended).
	FairyLight=0xFFE42D00,
	// If you are using no color correction, use this
	FairyLightNCC=0xFF9D2A00,
	PureWhite=0x000000FF,
	RedWhite=0xFF0000FF,
	GreenWhite=0x00FF00FF,
	BlueWhite=0x0000FFFF,
	HalfRedWhite=0x7F0000FF,
	HalfGreenWhite=0x007F00FF,
	HalfBlueWhite=0x00007FFF

} HTMLColorCode;


//typedef struct s_HTMLColor {
typedef struct {	
	uint32_t color;
	char name[30];
} HTMLColor;


// Point users to here for naming & colors https://www.w3schools.com/colors/colors_names.asp
// use strcasecmp() to avoid case comparison in the dictionary
#define COLOR_DICTIONARY_ELEMENTS			153

const HTMLColor ColorDictionary[COLOR_DICTIONARY_ELEMENTS] {
	{AliceBlue, "AliceBlue" },
	{Amethyst, "Amethyst" },
	{AntiqueWhite, "AntiqueWhite" },
	{Aqua, "Aqua" },
	{Aquamarine, "Aquamarine" },
	{Azure, "Azure" },
	{Beige, "Beige" },
	{Bisque, "Bisque" },
	{Beige, "Beige" },
	{Black, "Black" },
	{BlanchedAlmond, "BlanchedAlmond" },
	{Blue, "Blue" },
	{BlueViolet, "BlueViolet" },
	{BlueWhite, "BlueWhite"},
	{Brown, "Brown" },
	{BurlyWood, "BurlyWood" },
	{CadetBlue, "CadetBlue" },
	{Chartreuse, "Chartreuse" },
	{Chocolate, "Chocolate" },
	{Coral, "Coral" },
	{CornflowerBlue, "CornflowerBlue" },
	{Cornsilk, "Cornsilk" },
	{Crimson, "Crimson" },
	{Cyan, "Cyan" },
	{DarkBlue, "DarkBlue" },
	{DarkCyan, "DarkCyan" },
	{DarkGoldenrod, "DarkGoldenrod" },
	{DarkGray, "DarkGray" },
	{DarkGrey, "DarkGrey" },
	{DarkGreen, "DarkGreen" },
	{DarkKhaki, "DarkKhaki" },
	{DarkMagenta, "DarkMagenta" },
	{DarkOliveGreen, "DarkOliveGreen" },
	{DarkOrange, "DarkOrange" },
	{DarkOrchid, "DarkOrchid" },
	{DarkRed, "DarkRed" },
	{DarkSalmon, "DarkSalmon" },
	{DarkSeaGreen, "DarkSeaGreen" },
	{DarkSlateBlue, "DarkSlateBlue" },
	{DarkSlateGray, "DarkSlateGray" },
	{DarkSlateGrey, "DarkSlateGrey" },
	{DarkTurquoise, "DarkTurquoise" },
	{DarkViolet, "DarkViolet" },
	{DeepPink, "DeepPink" },
	{DeepSkyBlue, "DeepSkyBlue" },
	{DimGray, "DimGray" },
	{DimGrey, "DimGrey" },
	{DodgerBlue, "DodgerBlue" },
	{FairyLight, "FairyLight" },
	{FairyLightNCC, "FairyLightNCC" },
	{FireBrick, "FireBrick" },
	{FloralWhite, "FloralWhite" },
	{ForestGreen, "ForestGreen" },
	{Fuchsia, "Fuchsia" },
	{Gainsboro, "Gainsboro" },
	{GhostWhite, "GhostWhite" },
	{Gold, "Gold" },
	{Goldenrod, "Goldenrod" },
	{Gray, "Gray" },
	{Grey, "Grey" },
	{Green, "Green" },
	{GreenWhite, "GreenWhite" },
	{GreenYellow, "GreenYellow" },
	{HalfBlueWhite, "HalfBlueWhite"},
	{HalfGreenWhite, "HalfGreenWhite"},
	{HalfRedWhite, "HalfRedWhite"},
	{Honeydew, "Honeydew" },
	{HotPink, "HotPink" },
	{IndianRed, "IndianRed" },
	{Indigo, "Indigo" },
	{Ivory, "Ivory" },
	{Khaki, "Khaki" },
	{Lavender, "Lavender" },
	{LavenderBlush, "LavenderBlush" },
	{LawnGreen, "LawnGreen" },
	{LemonChiffon, "LemonChiffon" },
	{LightBlue, "LightBlue" },
	{LightCyan, "LightCyan" },
	{LightGoldenrodYellow, "LightGoldenrodYellow" },
	{LightGreen, "LightGreen" },
	{LightGrey, "LightGrey" },
	{LightPink, "LightPink" },
	{LightSalmon, "LightSalmon" },
	{LightSeaGreen, "LightSeaGreen" },
	{LightSkyBlue, "LightSkyBlue" },
	{LightSlateGray, "LightSlateGray" },
	{LightSlateGrey, "LightSlateGrey" },
	{LightSteelBlue, "LightSteelBlue" },
	{Lime, "Lime" },
	{LimeGreen, "LimeGreen" },
	{Linen, "Linen" },
	{Magenta, "Magenta" },
	{Maroon, "Maroon" },
	{MediumAquamarine, "MediumAquamarine" },
	{MediumBlue, "MediumBlue" },
	{MediumOrchid, "MediumOrchid" },
	{MediumPurple, "MediumPurple" },
	{MediumSeaGreen, "MediumSeaGreen" },
	{MediumSlateBlue, "MediumSlateBlue" },
	{MediumSpringGreen, "MediumSpringGreen" },
	{MediumTurquoise, "MediumTurquoise" },
	{MediumVioletRed, "MediumVioletRed" },
	{MintCream, "MintCream" },
	{MistyRose, "MistyRose" },
	{Moccasin, "Moccasin" },
	{NavajoWhite, "NavajoWhite" },
	{Navy, "Navy" },
	{OldLace, "OldLace" },
	{Olive, "Olive" },
	{OliveDrab, "OliveDrab" },
	{Orange, "Orange" },
	{OrangeRed, "OrangeRed" },
	{Orchid, "Orchid" },
	{PaleGoldenrod, "PaleGoldenrod" },
	{PaleGreen, "PaleGreen" },
	{PaleTurquoise, "PaleTurquoise" },
	{PaleVioletRed, "PaleVioletRed" },
	{PapayaWhip, "PapayaWhip" },
	{PeachPuff, "PeachPuff" },
	{Peru, "Peru" },
	{Pink, "Pink" },
	{Plaid, "Plaid" },
	{Plum, "Plum" },
	{PowderBlue, "PowderBlue" },	
	{PureWhite, "PureWhite" },
	{Purple, "Purple" },
	{Red, "Red" },
	{RedWhite, "RedWhite" },
	{RosyBrown, "RosyBrown" },
	{SaddleBrown, "SaddleBrown" },
	{Salmon, "Salmon" },
	{SandyBrown, "SandyBrown" },
	{SeaGreen, "SeaGreen" },
	{Seashell, "Seashell" },
	{Sienna, "Sienna" },
	{Silver, "Silver" },
	{SkyBlue, "SkyBlue" },
	{SlateBlue, "SlateBlue" },
	{SlateGray, "SlateGray" },
	{SlateGrey, "SlateGrey" },
	{Snow, "Snow" },
	{SpringGreen, "SpringGreen" },
	{SteelBlue, "SteelBlue" },
	{Tan, "Tan" },
	{Teal, "Teal" },
	{Thistle, "Thistle" },
	{Tomato, "Tomato" },
	{Violet, "Violet" },
	{Wheat, "Wheat" },
	{White, "White" },
	{WhiteSmoke, "WhiteSmoke" },
	{Yellow, "Yellow" },
	{YellowGreen, "YellowGreen" }
};

extern const HTMLColor ColorDictionary[COLOR_DICTIONARY_ELEMENTS];


// Travailler avec la classe ARRAY et BITSET ?
class CRGBW;


class CRGBW8 {
public:
	CRGBW8();
	~CRGBW8();
	CRGBW8(uint32_t color);
	CRGBW8(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0);
	void print(void);
	CRGBW8 operator= (const uint32_t color);
	CRGBW8 operator* (float val);
	CRGBW8 operator* (int val);
	CRGBW8 operator+ (const CRGBW8 &other);
	void operator+= (const CRGBW8 &other);
	void operator*= (float value);
	void operator*= (int value);
	bool operator== (const CRGBW8& color);
	bool operator!= (const CRGBW8& color);
	bool operator!= (const uint32_t color);
	
	// Accessor
	uint8_t operator[] (int channel) const;
	// Mutator
	uint8_t& operator[] (int channel);
	
	bool operator! () const; // devrait etre const
	bool notBlack() const;
	bool isBlack() const;
	bool isGrey() const;
	bool isWhite() const;
	uint32_t toInt() const;
	uint32_t toIntWRGB() const;
	void Black();
	CRGBW8 blend(CRGBW8& top, CRGBW8& bottom, float alpha_top);
	void blend(const CRGBW8& other, float amount);
	void blend(const CRGBW8& other, int x);
	CRGBW8 blendThis(const CRGBW8& other, int x);
	void blendPercent(const CRGBW8& other, int amount);
	void randColor();
	void randomize(int amount);
	void randomColor(bool normalize);
	void GRB();

	uint8_t r, g, b, w;
};

CRGBW8 randomColor(bool normalize = false);
CRGBW8 alphaBlend(CRGBW8 &top, CRGBW8 &bottom, float alpha_top);
CRGBW8 alphaBlend(CRGBW8 &top, CRGBW8 &bottom, int alpha_top);
CRGBW8 wheel(uint8_t WheelPos);
CRGBW8 wheelHue(int angle, int sat = 255, int brightness = 255);
CRGBW8 wheelHue(hsv& HSV);
CRGBW8 getColorFromDictionary(char *name);
uint8_t beforeOrAfter(char *compareStr, char *refStr);


/* CIE table generated using Jared Sanson's cie1931.py
 * from http://jared.geek.nz/2013/feb/linear-led-pwm
 */
static const uint8_t cie_lut[256] = {
	0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
	2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
	3, 4, 4, 4, 4, 4, 4, 5, 5, 5,
	5, 5, 6, 6, 6, 6, 6, 7, 7, 7,
	7, 8, 8, 8, 8, 9, 9, 9, 10, 10,
	10, 10, 11, 11, 11, 12, 12, 12, 13, 13,
	13, 14, 14, 15, 15, 15, 16, 16, 17, 17,
	17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
	22, 23, 23, 24, 24, 25, 25, 26, 26, 27,
	28, 28, 29, 29, 30, 31, 31, 32, 32, 33,
	34, 34, 35, 36, 37, 37, 38, 39, 39, 40,
	41, 42, 43, 43, 44, 45, 46, 47, 47, 48,
	49, 50, 51, 52, 53, 54, 54, 55, 56, 57,
	58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
	68, 70, 71, 72, 73, 74, 75, 76, 77, 79,
	80, 81, 82, 83, 85, 86, 87, 88, 90, 91,
	92, 94, 95, 96, 98, 99, 100, 102, 103, 105,
	106, 108, 109, 110, 112, 113, 115, 116, 118, 120,
	121, 123, 124, 126, 128, 129, 131, 132, 134, 136,
	138, 139, 141, 143, 145, 146, 148, 150, 152, 154,
	155, 157, 159, 161, 163, 165, 167, 169, 171, 173,
	175, 177, 179, 181, 183, 185, 187, 189, 191, 193,
	196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
	218, 220, 223, 225, 228, 230, 232, 235, 237, 240,
	242, 245, 247, 250, 252, 255,
};

#define RGB8(r,g,b) 	(CRGBW8(r,g,b))
#define RGBW8(r,g,b,w)	(CRGBW8(r,g,b,w))
#define getRed(color)  	(color.r)
#define getGreen(color)	(color.g)
#define getBlue(color)	(color.b)
#define getWhite(color)	(color.w)

#define MAX_RGB_LINE_WIDTH	1920




#endif /* __COLORS_H__ */
