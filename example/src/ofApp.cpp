#include "ofApp.h"

void ofApp::setup()
{
    cout << "Omron D6T-44L-06 test application." << endl;
	ofSetFrameRate(10);
	
	sensorPtr = 0;
    try {
        sensorPtr = new ofxOmronD6T();
    } catch ( ofxOmronD6TException& e ) {
        cout << e.what() << endl;
    }
	
	bShowTemps = true;
	bShowInstr = true;
}

void ofApp::update()
{
	int16_t *measurements = 0;
    measurements = sensorPtr->measure();
	for ( int i=0; i<16; i++ ) {
		//cout << measurements[i] << ", ";
		// Store float temperature in an array.
		temp_cel[i] = (float)measurements[i+1]*0.1f;
		// Map values to int range assuming that
		// the maximum temperature is 50.0f deg celsius.
		//int rgb_map = (int)(temp_cel[i]/50.0f*255.0f);
		// mapping the range from 10 to 40 deg cels
		int rgb_map = (int)((temp_cel[i]-10.0f)/30.0f*255.0f);
		// Set color for visual output
		colors[i] = ofColor(rgb_map,0,0);
	}
	//cout << endl;
}

void ofApp::draw()
{
    // draw 4x4 grid
    int rectWidth = (int)((float)ofGetWidth() / 4.0f);
    int rectHeight = (int)((float)ofGetHeight() / 4.0f);
    for ( int y=0; y<4; y++ ) {
    	for ( int x=0; x<4; x++ ) {
    		ofPushStyle();
    		ofSetColor( colors[y*4+x] );
			ofRect(x*rectWidth, y*rectHeight, rectWidth, rectHeight);
			ofPopStyle();
			// Show temperatures human readable
			if ( bShowTemps ) {
				stringstream ss;
				int pnum = y*4+x;
				ss << "P" << pnum << " " << temp_cel[y*4+x] << " C";
				ofDrawBitmapStringHighlight( ss.str(),ofPoint(x*rectWidth+10,y*rectHeight+20) );
			}
		}
    }
    
    // Show instructions
    if ( bShowInstr ) {
		stringstream is;
		is << "Press f to toggle fullscreen.\n";
		is << "Press t to toggle temperatures.\n";
		is << "Press i to toggle this info.";
		ofDrawBitmapStringHighlight( is.str(),ofPoint(10,ofGetHeight()-40) );
	}
}

void ofApp::keyPressed(int key)
{
	if ( key == 'f' ) {
		// Toggle fullscreen.
		ofToggleFullscreen();
	} else if ( key == 't'  ) {
		// Toogle show temperatures
		bShowTemps = !bShowTemps;
	} else if ( key == 'i' ) {
		// Toggle info
		bShowInstr = !bShowInstr;
	}
}
