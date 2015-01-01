#include "cv.h"
#include "highgui.h"
#include "markov.cpp"



using namespace cv;
using namespace std;



int main(int argc, char *argv[]){

Mat image  = imread(argv[1],0);
Mat mask = imread(argv[2],0);

MarkovRandomField mrf;
mrf.Mrf(image, mask);
namedWindow("Output");
imshow("Output",mrf.MrfOutput);
waitKey(0);
return 0;
}
