#include "process_frame.h"

ofxCvGrayscaleImage frame_threshold;
unsigned char* pixel;
IplImage* pixel_ellipse;
bool first_frame = true;
Point corneal_reflection(0, 0);
int corneal_reflection_radius = 0;
Point pupil;
vector<double> intensity_factor_horizontal(FRAMEH);
vector<double> avg_intensity_horizontal(FRAMEH);
int number_inliers = 0;

// deafult values for Starburst alghoritm
int edge_threshold = 10;
int rays = 18; 
int minimum_features = 10; 
int cr_window_size = 301;

/** process the frame
 * @param frame: new frame captured
 * @param start: start point for the algorithm
 * @param image: fundamental points' image
*/
void Process(ofxCvGrayscaleImage& frame, Point& start, ofxCvColorImage& image)
{
	int* inliers_index;
	CvSize ellipse_axis;
	static int lost_frame_number = 0;

	/*cout << "process, start_point: " << start << endl; // debug*/

	// reset fundamentals points' image
	image.set(0);

	// gaussian filter with kernel 5
	frame.blurGaussian(5);

	// extract pixels from frame
	pixel = (unsigned char*)frame.getCvImage()->imageData;

	// extract IplImage from fundamentals points' image
	pixel_ellipse = image.getCvImage();

	ReduceLineNoise(pixel);

	// remove corneal reflections
	for (int i = 0; i < NUM_LED; i++)
	{
		RemoveCornealReflection(frame, frame_threshold, start, cr_window_size, (int)(FRAMEH / 10), corneal_reflection,
			corneal_reflection_radius);

		DrawCross(pixel_ellipse, corneal_reflection, CROSS_LEN, YELLOW);
	}

	StarburstPupilContourDetection(pixel, FRAMEW, FRAMEH, edge_threshold, rays, minimum_features, start);

	// extract inliers indexes
	inliers_index = PupilFittingInliers(pixel, FRAMEW, FRAMEH, number_inliers);

	// get ellipse and pupil from PupilFittingInliers parameters
	ellipse_axis.width = (int)pupil_parameters[0];
	ellipse_axis.height = (int)pupil_parameters[1];
	pupil.x = (int)pupil_parameters[2];
	pupil.y = (int)pupil_parameters[3];

	DrawCross(pixel_ellipse, pupil, CROSS_LEN, BLUE);

	/*cout << "Ellipse: a=" << pupil_parameters[0] << ", b=" << pupil_parameters[1] << ", cx=" << pupil_parameters[2]
		<< ", cy= " << pupil_parameters[3] << ", theta=" << pupil_parameters[4] << "; number_inliers=" 
		<< number_inliers << endl; // debug*/

	bool is_inliers;
	
	for (unsigned i = 0; i < edge_point.size(); i++)
	{
		is_inliers = false;
		for (int j = 0; j < number_inliers; j++)
		{
			if (i == inliers_index[j])
			{
				is_inliers = true;
				break;
			}
		}

		Point_<double> *edge_double = edge_point.at(i);
		Point edge;
		edge.x = (int)edge_double->x;
		edge.y = (int)edge_double->y;
		
		if (is_inliers)
		{
			DrawCross(pixel_ellipse, edge, CROSS_LEN, GREEN);
		}
		else
		{
			DrawCross(pixel_ellipse, edge, CROSS_LEN, RED);
		}
	}
	std::free(inliers_index);

	// verify that ellipse is not imaginary
	if (ellipse_axis.width > 0 && ellipse_axis.height > 0)
	{
		start = pupil; // set the start point for next frame
		
		// draw ellipse in both images
		cvEllipse(frame.getCvImage(), pupil, ellipse_axis, -pupil_parameters[4] * 180 / PI, 0, 360, WHITE, 2);
		cvEllipse(pixel_ellipse, pupil, ellipse_axis, -pupil_parameters[4] * 180 / PI, 0, 360, GREEN, 2);

		lost_frame_number = 0;
	}
	else
	{
		lost_frame_number++;
	}

	if (lost_frame_number > 5)
	{
		// reset start point
		start.x = FRAMEW / 2;
		start.y = FRAMEH / 2;
	}
}

/** reduce line noise: C(i,k) = beta*Iaverage(i,k) + (1-beta)*C(i-1,k)
 * @param pixel: pixels of the frame
*/
void ReduceLineNoise(unsigned char* pixel)
{
	double beta_complementare = 1 - beta;
	int adjustment;

	CalculateAvgIntenistyHorizontal(pixel);

	// for the first frame copy avg_intensity_horizontal in intensity_factor_horizontal
	if (first_frame) 
	{
		intensity_factor_horizontal = avg_intensity_horizontal;
		first_frame = false;
	}

	for (unsigned j = 1; j < FRAMEH; j++)
	{
		intensity_factor_horizontal[j] = avg_intensity_horizontal[j] * beta + intensity_factor_horizontal[j - 1] * beta_complementare;
		adjustment = (int)(intensity_factor_horizontal[j] - avg_intensity_horizontal[j]);
	
		for (unsigned i = 0; i < FRAMEW; i++) 
			*pixel = FIX_PIXEL(*pixel + adjustment);
	}
}

/** calculate average horizontal intensity for frame
* @param pixel: pixels of the frame
*/
void CalculateAvgIntenistyHorizontal(const unsigned char* pixel)
{
	int sum;

	for (unsigned j = 0; j < FRAMEH; j++)
	{
		sum = 0;

		for (unsigned i = 0; i < FRAMEW; i++) 
			sum += *pixel;

		avg_intensity_horizontal[j] = (double)sum / FRAMEW;
	}
}


/** draw a cross
 * @param image: image where to draw the cross
 * @param center: center of the cross
 * @param cross_length: length of the cross
 * @param color: color of the cross
*/
void DrawCross(IplImage* image, Point center, int cross_length, Scalar color)
{
	if (cross_length <= 0)
		return;

	Point pt1, pt2, pt3, pt4;

	// horizontal segment
	pt1.x = center.x - cross_length;
	pt1.y = center.y;
	pt2.x = center.x + cross_length;
	pt2.y = center.y;

	// vertical segment
	pt3.x = center.x;
	pt3.y = center.y - cross_length;
	pt4.x = center.x;
	pt4.y = center.y + cross_length;

	// draw segments
	cvLine(image, pt1, pt2, color);
	cvLine(image, pt3, pt4, color);
}