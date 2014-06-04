#ifndef H_OF_APP
#define H_OF_APP

#include "ofMain.h"
#include "ofxOmronD6T.h"

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	ofxOmronD6T* sensorPtr;
	int address;
	string i2c_bus;
	int model;
    
	ofColor colors[16];
	float temp_cel[16];
	
	// Display settings.
	bool bShowTemps;
	bool bShowInstr;
};

#endif
