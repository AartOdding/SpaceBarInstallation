#include "ofMain.h"
#include "ofApp.h"
#include "ofApp2.h"



int main()
{
	ofGLWindowSettings settings;
	settings.setGLVersion(4, 1);
	auto window = ofCreateWindow(settings);
	//ofRunApp(new ofApp());
	ofRunApp(new ofApp2());
}
