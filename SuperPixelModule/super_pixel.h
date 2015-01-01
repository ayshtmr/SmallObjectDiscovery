//
// This ia an implementation for image segmentation using super pixel.
// The implementation is based on SLIC super pixel:
// Radhakrishna Achanta, Appu Shaji, Kevin Smith,
// Aurelien Lucchi, Pascal Fua, and Sabine Susstrunk,
// "SLIC Superpixels",
// EPFL Technical Report no. 149300, June 2010.
//
// The idea of super pixel is to segment image into small
// patches(segments)(super pixels) based on:
// 1. the number of expected super pixels given by the user.
// 2. the number of expected size of super pixels given by the user.
//
// For image segmentation, the algorithum takes both the pixel color and
// pixel geometric position into consideration.

#ifndef DOTORG_CHILDPROTECTION_BEDSPREADS_PRODUCTION_GOOGLE_SIDE_SEGMENTATION_SUPER_PIXEL_SUPER_PIXEL_H_
#define DOTORG_CHILDPROTECTION_BEDSPREADS_PRODUCTION_GOOGLE_SIDE_SEGMENTATION_SUPER_PIXEL_SUPER_PIXEL_H_

#include <string>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cv.h"

using namespace cv;

// This class implements super pixel for image segmentation.
// When the user input the image for segmentation and the number
// of expected segments (or the expected size of super pixel),
// it returns user a segmentation result map (the same size as the input image),
// in which the element value indicates the segmentation index.
// Also, we use a struct "Segment" to carry detail information for the
// segmentation result. In "Segment", it describes the geometric center of the
// current segment as well as well shows the number of pixels in this segment.
// The class "SuperPixels" also returns user a vector of "Segment". The vector
// size is exactly the number of segmented areas.
//
// Sample usage:
//
//   // Initialization.
//   const string image_path = "/path/for/your/input/image";
//   scoped_ptr<SuperPixel> sp(new SuperPixel(image_path));
//
//   // Usage 1: by defining number of expected segments:
//   int const kExpectedNumber = 300;
//   // Note: the actual segments number after segmentation is close to
//   // the input one but sometimes not exactly equal to.
//   // We still need to give a parameter M(kWeightM), which defines
//   // the weighting between color feature and geometric feature for
//   // segmentation. As the original paper suggests, M can
//   // be in the range [1, 20]. They simply chosen 10 in their experiments.
//   int const kWeightM = 10;
//   // Image segmentation.
//   sp->SegmentNumber(kExpectedNumber, kWeightM);
//   // Get the final number of segments:
//   int num_segments = sp->num_segments();
//   // Get the vector which describes the segments:
//   vector<Segment> segments = sp->segments();
//   // Get the segmentation map which denotes the segmentation result:
//   // If you want to know pixel position (row, col)'s segmentation result,
//   // you can access segmentation_map[row * width_ + col].
//   scoped_ptr<int> segmentation_map(sp->segmentation_map());
//   // Do something with the segmentation result...
//   // Draw the segmentation result (using red border).
//   CvScalar drawing_color = CV_RGB(255, 0, 0);
//   IplImage* contour = NULL;
//   sp->DrawContours(drawing_color, contour);
//   // Save drawing result using OpenCV function.
//   cvSaveImage("./seg_contour.jpg", contour);
//   cvReleaseImage(&contour);
//
//   // Usage 2: by defining the expected size of super pixel:
//   int const kExpectedSideLength = 50;
//   // In this case, we are expecting the super pixel is around 50 x 50.
//   int const kWeightM = 10;
//   sp->SegmentSize(kExpectedSideLength, kWeightM);
//   // Get the final number of segments:
//   int num_segments = sp->segments();
//   // Get the vector which describes the segments:
//   vector<Segment> segments = sp->segments();
//   // Get the segmentation map which denotes the segmentation result:
//   scoped_ptr<int> segmentation_map(sp->segmentation_map());
//   // Do something with the segmentation result...


namespace ncmec {
//
// This struct is only used for carry data. It shows the segment
// area information.
//
// center: The geometric center of the current segment.
// pixel_num: Number of pixels in the current segment.
//
struct Segment {
  CvPoint center;
  int pixel_num;
};

//
// Main class of image segmentation by super pixel.
//
class SuperPixel {
 public:
  //
  // Constructors
  //
  // Parameters:
  //
  // image_path: the path points to the image for segmentation.
  //
  // image: a OpenCV IplImage pointer points to the image for segmentation.
  //
  explicit SuperPixel(const std::string& image_path) {
    image_ = cvLoadImage(image_path.c_str(), CV_LOAD_IMAGE_COLOR);
    num_segments_ = -1;
    width_ = image_->width;
    height_ = image_->height;
    segmentation_map_ = NULL;
    l_values_ = NULL;
    a_values_ = NULL;
    b_values_ = NULL;
    gradients_ = NULL;
  }
  explicit SuperPixel(const IplImage* image) {
    image_ = cvCloneImage(image);
    num_segments_ = -1;
    width_ = image_->width;
    height_ = image_->height;
    segmentation_map_ = NULL;
    l_values_ = NULL;
    a_values_ = NULL;
    b_values_ = NULL;
    gradients_ = NULL;
  }
  //
  // Destructor
  //
  // We need to use cvReleaseImage to release the IplImage pointer.
  //
  ~SuperPixel() {
    if (image_) {
      cvReleaseImage(&image_);
      image_ = NULL;
    }
    if (segmentation_map_) {
      delete [] segmentation_map_;
      segmentation_map_ = NULL;
    }
    if (l_values_) {
      delete [] l_values_;
      l_values_ = NULL;
    }
    if (a_values_) {
      delete [] a_values_;
      a_values_ = NULL;
    }
    if (b_values_) {
      delete [] b_values_;
      b_values_ = NULL;
    }
    if (gradients_) {
      delete [] gradients_;
      gradients_ = NULL;
    }
  }

  // Image segmentation functions.
  //
  // Parameters:
  //
  // expected_seg_num: expected number of segments.
  //
  // expected_seg_size: expected side length of super pixels.
  //
  // weight_m: weighting parameter between color feature hint and geometric
  //           position hint.
  //
  // A: by defining the expected number of segments(clusters).
  bool SegmentNumber(const int& expected_seg_num, const float& weight_m);
  // B: by defining the expected size of super pixel.
  // The expected super pixel  will be (expected_seg_size x expected_seg_size).
  bool SegmentSize(const int& expected_seg_size, const float& weight_m);
  // Testing function.
  // Draw the contours of segmented areas.
  //
  // Parameters:
  //
  // drawing_color: The color for the contours(segments borders).
  //
  // contour: The drawing result.
  //
  void DrawContours(const CvScalar& drawing_color, const std::string& save_path, const std::string& save_pathskel,int &nmaxcluster,Mat &labelArray);
  Mat labelledImage(int label);

  //
  // Accessors.
  //
  // Returns the number of segments.
  int num_segments() const {
    return num_segments_;
  }
  // Returns the segmentation result.
  // segmentation_map_ has the same number of pixels with image_.
  // But it is 1-d array instead of 2-d matrix.
  const int* segmentation_map() const {
    return segmentation_map_;
  }
  // Returns the segment's details such as geometric center, number of pixels.
  const std::vector<Segment> segments() const {
    return segments_;
  }

 private:
  // Image for segmentation. We read it bu using OpenCV's IplImage.
  IplImage* image_;
  // Number of segmented areas after segmentation.
  int num_segments_;
  // Segmentation result.
  int* segmentation_map_;
  // Input image's width.
  int width_;
  // Input image's height.
  int height_;
  // Clustering (segmentation) results.
  std::vector<Segment> segments_;
  //
  // The algorithum use CIELAB color space.
  // We turn the RGB image into LAB color space and store the l, a, b values.
  // Besides, the original 2-d image data is transformed into 1-d array.
  // l space values for the image data.
  float* l_values_;
  // a space values for the image data.
  float* a_values_;
  // b space values for the image data.
  float* b_values_;
  // Float array stores the gradient on lab color space.
  float* gradients_;
  //
  // The segmentation problem is actually a clustering problem.
  // Like k-means, We do the clustering by setting several initial
  // seeds(centers), and iteratively dispatch pixels to these centers.
  // Following are the vectors describing thses centers.
  // The l values of current centers.
  std::vector<float> k_centers_l_;
  // The a values of current centers.
  std::vector<float> k_centers_a_;
  // The b values of current centers
  std::vector<float> k_centers_b_;
  // The x positions of current centers.
  std::vector<int> k_centers_x_;
  // The y positions of current centers.
  std::vector<int> k_centers_y_;

  //
  // Useful functions for segmentation.
  //
  // Super pixel use CIELAB color space, we first need to
  // Convert the image color space from BGR(OpenCv) to CIELAB.
  // Meanwhile, the 2-D image is vectorized into 1-D float arrays.
  void BGR2LAB();
  //
  // The super pixel algorithum first dispatch 'seeds' ( = expected_seg_num)
  // as the initial clustering centers. They suggests to perturb these seeds
  // to a local lowest gradient position.
  // This function detects gradient map to perturb seeds.
  void DetectGradients();
  //
  // Get the initial centers based on given super pixel size (side length).
  void GetInitialCenters(const int& expected_seg_size);
  //
  // Super pixel clustering. Need post-processing for enforcing connectivity.
  //
  // Parameters:
  //
  // expected_seg_size: expected size (side length) of segments.
  //
  // weigth_m: weighting parameter between color feature hint and geomotric
  //           position hint for clustering.
  //
  // temp_segmentation_map: the temp result after several iteration for pixel
  //                        clustering. But we still need post processing to
  //                        enforce the connectivity of segmentation results.
  void ClusteringIteration(const int& expected_seg_size,
                           const float& weight_m,
                           int* temp_segmentation_map);
  //
  // Find next connected components(pixel) which belongs to the same cluster.
  // Function is called recursively to get the size of connected area cluster.
  //
  // Parameters:
  //
  // temp_segmentation_map: the preliminary clustering result generated by
  // "ClusteringIteration" function.
  //
  // row_index: row number of the current pixel.
  //
  // col_index: col number of the current pixel.
  //
  // segment_index: current segment index.
  //              We are counting its number of components(pixels).
  //
  // x_pos: store the pixels' x positions.
  //
  // y_pos: store the pixels' y positions.
  //
  // num_count: counting the number of pizels belong to current segment.
  void FindNext(const int* temp_segmentation_map,
                const int& row_index,
                const int& col_index,
                const int& segment_index,
                int* x_pos,
                int* y_pos,
                int* num_count);
  //
  // Post-processing. Enforce connectivity.
  // At the end of PerformSuperPixel(...), a few stray labels may remian,
  // that is, a few pixels in the vicinity of a large segment having the same
  // label but not connected to it. We enforce connectivity in the last step by
  // relabeling disjoint segments with the labels of the largest neighboring
  // cluster.
  void EnforceConnectivity(const int* temp_segmentation_map,
                           const int& expected_seg_size);
};
}  // namespace ncmec
#endif  // DOTORG_CHILDPROTECTION_BEDSPREADS_PRODUCTION_GOOGLE_SIDE_SEGMENTATION_SUPER_PIXEL_SUPER_PIXEL_H_
