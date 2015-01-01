#include "cv.h"
#include "highgui.h"
#include <fstream>
#include <cstring>

#define MINL 370
#define MAXL 395

using namespace cv;
using namespace std;


int main(int argc , char *argv[]){

	for(int i = MINL ; i< MAXL ; i++){
		stringstream ss;	
		ss<<i;
		string si = "dataset/" + ss.str() + ".jpg";
		string st = "datasetresize/" + ss.str() + ".jpg";	

    		char *s = new char[si.length() + 1];
    		strcpy(s, si.c_str());
		Mat image  = imread(si,1);
		resize(image,image,Size(480,360));
		imwrite(st,image);
		cout<<st<<endl;
	}

}
