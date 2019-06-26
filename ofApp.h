#pragma once

#include <iostream>
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "..\mylib\process_frame.h"


class ofApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	// variables
	int MARGIN_IMAGE = 40; // left and upper margin for images
	ofVideoGrabber video;
	ofxCvColorImage frame_rgb;
	ofxCvGrayscaleImage frame_gs;
	ofxCvColorImage image_points; // image with fundamental points drawn
	Point start_point = { FRAMEW / 2, FRAMEH / 2 }; // default start point for the algorithm: center of the image
	bool clicked = false;
};
