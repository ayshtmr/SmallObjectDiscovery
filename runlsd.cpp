#include <iostream>
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <cv.h>
#include "highgui.h"


using namespace std;
using namespace cv;

//gs -sDEVICE=jpeg -dJPEGQ=100 -dNOPAUSE -dBATCH -dSAFER -r300 -sOutputFile=myfile.jpg myfile.eps
//mogrify -trim -resize 800x600 myfile.jpg

//give permissions to lsd binary!! 	


int main(int argc , char *argv[]){

char buffer[200];
char ins1[200];
char ins2[200];  
char s1[20] = "-P";
char s2[20] = "floor.result.eps";
namedWindow("Image");

	for(int i=1;i<3;i++){
	stringstream ss;
	stringstream sresult;
	stringstream sconvert;

	ss<<i;
	sresult<<i;
	sconvert<<i;

    string s = "./" + ss.str() + ".pgm";
    //string sr = "mogrify -trim -resize 640x480 /home/ayush/floor_seg/in_data/"+ sresult.str() + ".jpg";
    //string sc = "gs -sDEVICE=jpeg -dJPEGQ=100 -dNOPAUSE -dBATCH -dSAFER -r300 -sOutputFile=/home/ayush/floor_seg/in_data/" + sresult.str() + ".jpg " + "floor.result.eps";

	//cout<<s<<endl;
	char *s3 = new char[s.length() + 1];
        strcpy(s3, s.c_str());
	
	// char *c1 = new char[sc.length() + 1];
 //        strcpy(c1, sc.c_str());

	// char *c2 = new char[sr.length() + 1];
 //        strcpy(c2, sr.c_str());
	
	

    string st = "./" + sresult.str() + ".txt";
    char *s4 = new char[st.length() + 1];
    strcpy(s4, st.c_str());
	//char s3[20] = {s};
	//char s4[20] = "floor.result.txt";
	//char c1[200]= "gs -sDEVICE=jpeg -dJPEGQ=100 -dNOPAUSE -dBATCH -dSAFER -r300 -sOutputFile=myfile.jpg floor.result.eps";
	//char c2[200]= "mogrify -trim -resize 640x480 myfile.jpg";


	// cout<<c1<<endl;
	// cout<<c2<<endl;	

	//printf("%s %s %s %s\n",s1,s2,s3,s4);
	sprintf(buffer, "./lsd %s %s %s %s",s1,s2,s3,s4);
	//sprintf(ins1, "%s",c1);
	//sprintf(ins2, "%s",c2);
	system(buffer);
	//system(ins1);
	//system(ins2);


	/*string output = "Result/"+ sresult.str() + ".jpg";
	Mat image = imread(output,0);
	imshow("Image",image);
	waitKey(0);*/
	waitKey(10);
	}
return 0;
}
