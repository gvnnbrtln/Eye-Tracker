#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() 
{

	//get back a list of devices
	vector<ofVideoDevice> devices = video.listDevices();

	// check device availability
	for (unsigned i = 0; i < devices.size(); i++)
	{
		if (devices[i].bAvailable)
			ofLogNotice() << devices[i].id << ": " << devices[i].deviceName;
		else
			ofLogNotice() << devices[i].id << ": " << devices[i].deviceName << " - unavailable ";
	}

	video.setDeviceID(1); // set ps3eye
	video.setDesiredFrameRate(FRAMERATE); // set the frame rate
	video.initGrabber(FRAMEW, FRAMEH);

	// frames allocation
	frame_rgb.allocate(FRAMEW, FRAMEH);
	frame_gs.allocate(FRAMEW, FRAMEH);
	frame_threshold.allocate(FRAMEW, FRAMEH);
	image_points.allocate(FRAMEW, FRAMEH);
	image_points.set(0); // set black background for fundamentals points' image

	ofBackground(50, 50, 50); // set window's background color
}

//--------------------------------------------------------------
void ofApp::update() 
{

	video.update();
	// image_points.setFromPixels((unsigned char*)image_points.getCvImage()->imageData, FRAMEW, FRAMEH); // aggiorno l'immagine dei punti fondamentali

	if (video.isFrameNew())
	{
		frame_rgb = video.getPixels(); // get the rgb frame from videograbber
		frame_gs = frame_rgb; // convert the rgb frame to grayscale frame

		if (clicked)
			Process(frame_gs, start_point, image_points); // process the frame
	}
}

//--------------------------------------------------------------
void ofApp::draw() 
{

	if (!clicked)
		ofDrawBitmapString("Left click on the center of pupil", MARGIN_IMAGE, MARGIN_IMAGE / 2);
	else
		ofDrawBitmapString("Tracking started", MARGIN_IMAGE, MARGIN_IMAGE / 2);

	frame_gs.draw(MARGIN_IMAGE, MARGIN_IMAGE, FRAMEW, FRAMEH);
	image_points.draw(2 * MARGIN_IMAGE + FRAMEW, MARGIN_IMAGE, FRAMEW, FRAMEH);

	ofDrawBitmapString("Fundamental points:", 2 * MARGIN_IMAGE + FRAMEW, MARGIN_IMAGE / 2);
	ofDrawBitmapString("Legend:", 2 * MARGIN_IMAGE + FRAMEW, 2 * MARGIN_IMAGE + FRAMEH);
	ofDrawBitmapString("yellow cross: corneal reflection", 2 * MARGIN_IMAGE + FRAMEW, 5 * MARGIN_IMAGE / 2 + FRAMEH);
	ofDrawBitmapString("blue cross: center of pupil", 2 * MARGIN_IMAGE + FRAMEW, 3 * MARGIN_IMAGE + FRAMEH);
	ofDrawBitmapString("green cross: inlier", 2 * MARGIN_IMAGE + FRAMEW, 7 * MARGIN_IMAGE / 2 + FRAMEH);
	ofDrawBitmapString("red cross: outlier", 2 * MARGIN_IMAGE + FRAMEW, 4 * MARGIN_IMAGE + FRAMEH);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	// capture start point 
	if (button == 0)
	{
		cout << "clicked point: (" << x << ", " << y << ")" << endl; // debug

		// verify that start point is within the image
		if ((x - MARGIN_IMAGE >= 0) && (x - MARGIN_IMAGE <= FRAMEW) && (y - MARGIN_IMAGE >= 0) && (y - MARGIN_IMAGE <= FRAMEH))
		{
			start_point.x = x - MARGIN_IMAGE;
			start_point.y = y - MARGIN_IMAGE;

			clicked = true;
		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

