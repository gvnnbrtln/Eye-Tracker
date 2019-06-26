#include "ransac.h"

vector<double> pupil_parameters(5, 0);
vector<Point_<double>*> edge_point;
vector<int> edge_intensity_diff;

/** get five random numbers
 * @param max: maximum number
 * @param random_num: array with random number
*/
void GetFiveRandomNum(int max, vector<int>& random_num)
{
	bool is_new;
	int r;

	if (max == 4)
	{
		for (unsigned i = 0; i < 5; i++)
			random_num[i] = i;

		return;
	}

	for (unsigned i = 0; i < 5; i++)
	{
		is_new = true;
		r = (int)((rand() * max)/ RAND_MAX);

		for (unsigned j = 0; j < i; j++)
		{
			if (r == random_num[j])
			{
				is_new = false;
				break;
			}
		}
			
		if (is_new) 
			random_num[i] = r;
	}
}

/** get edge mean
 * @return average intensity of edge pixels
*/
Point_<double> GetEdgeMean()
{
	double sumx = 0, sumy = 0;
	Point_<double> edge_mean;

	for (unsigned i = 0; i < edge_point.size(); i++)
	{
		sumx += edge_point[i]->x;
		sumy += edge_point[i]->y;
	}

	if (edge_point.size() > 0)
	{
		edge_mean.x = sumx / edge_point.size();
		edge_mean.y = sumy / edge_point.size();
	}
	else
	{
		edge_mean.x = -1;
		edge_mean.y = -1;
	}

	return edge_mean;
}

/** destroy edge point
*/
void DestroyEdgePoint()
{
	if (edge_point.size() > 0)
	{
		for (unsigned i = 0; i < edge_point.size(); i++)
			std::free(edge_point[i]);
	
		edge_point.clear();
	}
}

/** get ellipse parameters from conic parameters
 * @param conic_parameters: conic parameters
 * @param ellipse_parameters: ellipse parameters
*/
bool SolveEllipse(const vector<double>& conic_parameters, vector<double>& ellipse_parameters)
{
	// conic coefficients (ax^2 + bxy + cy^2 + dx + ey + f = 0)
	double a = conic_parameters[0];
	double b = conic_parameters[1];
	double c = conic_parameters[2];
	double d = conic_parameters[3];
	double e = conic_parameters[4];
	double f = conic_parameters[5];

	// get ellipse orientation
	double theta = atan2(b, a - c) / 2;

	// get scaled major/minor axes
	double ct = cos(theta);
	double st = sin(theta);
	double a_rot = a*ct*ct + b*ct*st + c*st*st; // a coefficient after theta rotation
	double c_rot = a*st*st - b*ct*st + c*ct*ct; // c coefficient after theta rotation

	// get translations
	// ellipse center coordinates in originary refernce system
	double cx = (2 * c*d - b*e) / (b*b - 4 * a*c);
	double cy = (2 * a*e - b*d) / (b*b - 4 * a*c);

	//get scale factor
	double val = a*cx*cx + b*cx*cy + c*cy*cy; // known term of translated ellipse
	double scale_inv = val - f;

	// verify that the ellipse is not imaginary
	if ((scale_inv / a_rot <= 0) || (scale_inv / c_rot <= 0)) 
	{
		ellipse_parameters.assign(ellipse_parameters.size(), 0);
		return false;
	}

	// get ellipse parameters
	ellipse_parameters[0] = sqrt(scale_inv / a_rot); // a axis
	ellipse_parameters[1] = sqrt(scale_inv / c_rot); // b axis
	ellipse_parameters[2] = cx; // abscissa of the center
	ellipse_parameters[3] = cy; // ordinate of the center
	ellipse_parameters[4] = theta; // rotation

	return true;
}

/** translate and scale edge points to obtain (0,0) center and average distance from origin equal to sqrt(2)
 * @param dis_scale: scale factor
 * @param normalized_center: normalized center
*/
Point_<double>* NormalizeEdgePoint(double& dis_scale, Point_<double>& normalized_center)
{
	double sumx = 0, sumy = 0;
	double sumdis = 0;
	Point_<double> *edge;

	for (unsigned i = 0; i < edge_point.size(); i++)
	{
		edge = edge_point.at(i);
		sumx += edge->x;
		sumy += edge->y;

		// sum of distances of edge points from origin
		sumdis += sqrt((double)(edge->x * edge->x + edge->y * edge->y)); 
	}

	dis_scale = (double) sqrt(2) * edge_point.size() / sumdis; // s variabile in normalized DLT
	normalized_center.x = (double)sumx / edge_point.size();
	normalized_center.y = (double)sumy / edge_point.size();
	
	Point_<double>* edge_point_normalized = (Point_<double>*)malloc(sizeof(Point_<double>)*edge_point.size());

	for (unsigned i = 0; i < edge_point.size(); i++)
	{
		edge = edge_point.at(i);
		// calculate normalized points: (x', y') = (s*x+tx, s*y+ty) where tx = -s*x_avg, ty = -s*y_avg
		edge_point_normalized[i].x = (edge->x - normalized_center.x) * dis_scale;
		edge_point_normalized[i].y = (edge->y - normalized_center.y) * dis_scale;
	}

	return edge_point_normalized;
}

/** denormalize ellipse parameters
 * @param parameters: denormalized parameters
 * @param normalized_parameters: normalized parameters
 * @param dis_scale: scale_factor
 * @param normalized_center: normalized center
*/
void DenormalizeEllipseParameters(vector<double>& parameters, const vector<double>& normalized_parameters, double dis_scale,
	Point_<double> normalized_center)
{
	//major and minor axis
	parameters[0] = normalized_parameters[0] / dis_scale;	
	parameters[1] = normalized_parameters[1] / dis_scale;
	//ellipse center
	parameters[2] = normalized_parameters[2] / dis_scale + normalized_center.x;	
	parameters[3] = normalized_parameters[3] / dis_scale + normalized_center.y;
}

/** find edge points
 * @param image: pixels of the frame
 * @param width: width of the frame
 * @param height: height of the frame
 * @param center: start point
 * @param dis: scale factor
 * @param angle_step: angle step
 * @param angle_normal: middle angle in the angle spread
 * @param angle_spread: angle amplitude to find edge points
 * @param threshold: threshold to detect edge points
*/
void LocateEdgePoints(unsigned char* image, int width, int height, Point_<double> center, int dis, double angle_step,
	double angle_normal, double angle_spread, int threshold)
{
	double angle;
	Point_<double> p;
	Point_<double>* edge;
	double dis_cos, dis_sin;
	int pixel_value1, pixel_value2;

	// search edge points moving along the ray identifed by angle from center 
	for (angle = angle_normal - angle_spread / 2 + 0.0001; angle < angle_normal + angle_spread / 2; angle += angle_step)
	{
		dis_cos = dis * cos(angle);
		dis_sin = dis * sin(angle);
		p.x = center.x + dis_cos;
		p.y = center.y + dis_sin;

		pixel_value1 = image[(int)(p.y) * width + (int)(p.x)];

		while (1)
		{
			p.x += dis_cos;
			p.y += dis_sin;

			// stay within the image
			if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height)
				break;

			pixel_value2 = image[(int)(p.y) * width + (int)(p.x)];

			// if the intensity derivative along the ray is bigger than threshold I probably found an edge point
			if (pixel_value2 - pixel_value1 > threshold)
			{
				edge = (Point_<double>*)malloc(sizeof(Point_<double>));
				// set edge point between the two pixels
				edge->x = p.x - dis_cos / 2;
				edge->y = p.y - dis_sin / 2;
			
				edge_point.push_back(edge);

				edge_intensity_diff.push_back(pixel_value2 - pixel_value1);
				break;
			}

			// the last pixel becomes the first pixel for the next iteration
			pixel_value1 = pixel_value2;
		}
	}
}

/** implementation of Starburst algorithm
 * @param image: pixels of the frame
 * @param width: width of the frame
 * @param height: height of the frame
 * @param threshold: threshold to detect edge points
 * @param N: number of rays along which search for the edge points
 * @param minimum_features: minimum number of edge points that are needed to fit the ellipse
 * @param start_point: start point
*/
void StarburstPupilContourDetection(unsigned char* image, int width, int height, int threshold, int N,
	int minimum_features, Point start_point)
{
	int dis = 7; // step for the moving along the rays
	double angle_spread = 100 * PI / 180;
	int loop_count = 0;
	double angle_step = 2 * PI / N;
	double new_angle_step;
	Point_<double>* edge;
	Point_<double> edge_mean;
	double angle_normal;
	Point_<double> c = start_point;

	while (threshold > 5 && loop_count <= 10)
	{
		edge_intensity_diff.clear();
		DestroyEdgePoint();
		while (edge_point.size() < minimum_features && threshold > 5)
		{
			edge_intensity_diff.clear();
			DestroyEdgePoint();

			// find a first set of edge points
			LocateEdgePoints(image, width, height, c, dis, angle_step, 0, 2 * PI, threshold);

			if (edge_point.size() < minimum_features)
				threshold -= 1;
		}

		if (threshold <= 5)
			break;

		// from the first set of edge points search for a second set
		int first_ep_num = edge_point.size();
		for (unsigned i = 0; i < first_ep_num; i++)
		{
			edge = edge_point.at(i);
			angle_normal = atan2(c.y - edge->y, c.x - edge->x);
			new_angle_step = angle_step * ((double)threshold / edge_intensity_diff.at(i));

			LocateEdgePoints(image, width, height, *edge, dis, new_angle_step, angle_normal,
				angle_spread, threshold);
		}

		loop_count++;
		edge_mean = GetEdgeMean();

		// if the mean is near the center edge points are detected
		if (fabs(edge_mean.x - c.x) + fabs(edge_mean.y - c.y) < 10)
			break;

		// set the edge mean as start point for the next iteration
		c.x = edge_mean.x;
		c.y = edge_mean.y;
	}

	if (loop_count > 10) 
	{
		DestroyEdgePoint();
		cout << "Error! edge points did not converge in " << loop_count <<  " iterations!" << endl;
		return;
	}

	if (threshold <= 5) 
	{
		DestroyEdgePoint();
		cout << "Error! Adaptive threshold is too low!" << endl;
		return;
	}
}

/** find the inliers which approsimate the ellipse 
 * @param image: pixels of the frame
 * @param width: width of the image
 * @param height: height of the image
 * @param return_max_inliers: maximum number of inliers
 * @return indexes of inliers 
*/
int* PupilFittingInliers(unsigned char* image, int width, int height, int& return_max_inliers)
{
	Point_<double> nor_center;
	double dis_scale;

	int ellipse_point_num = 5;	// number of points that are needed to fit an ellipse

	if (edge_point.size() < ellipse_point_num)
	{
		cout << "Error! " << edge_point.size() << " points are not enough to fit ellipse" << endl;
		pupil_parameters.assign(pupil_parameters.size(), 0);
		return_max_inliers = 0;
		return NULL;
	}

	// normalization
	Point_<double>* edge_point_nor = NormalizeEdgePoint(dis_scale, nor_center);

	// Ransac
	int *inliers_index = (int*)malloc(sizeof(int)*edge_point.size());
	int *max_inliers_index = (int*)malloc(sizeof(int)*edge_point.size());
	int ninliers = 0;
	int max_inliers = 0;
	int sample_num = 1000;
	int ransac_count = 0;
	double dis_threshold = 1.95959 * dis_scale;
	double dis_error;

	memset(inliers_index, int(0), sizeof(int)*edge_point.size());
	memset(max_inliers_index, int(0), sizeof(int)*edge_point.size());

	vector<int> rand_index(5);

	// matrixes for SVD: S = U*A*V
	Mat A(6, 6, DataType<double>::type); 
	Mat S, U, V;

	// set the last column of A to 1 and last row to 0
	for (unsigned i = 0; i < 6; i++)
	{
		A.at<double>(i, 5) = 1;
		A.at<double>(5, i) = 0;
	}

	vector<double> pd(6);
	int min_d_index;
	vector<double> conic_parameters(6, 0);
	vector<double> ellipse_parameters(5, 0);
	vector<double> best_ellipse_parameters(5, 0);
	double ratio;

	while (sample_num > ransac_count)
	{
		GetFiveRandomNum((edge_point.size() - 1), rand_index);

		for (unsigned i = 0; i < 5; i++)
		{
			A.at<double>(i, 0) = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].x;
			A.at<double>(i, 1) = edge_point_nor[rand_index[i]].x * edge_point_nor[rand_index[i]].y;
			A.at<double>(i, 2) = edge_point_nor[rand_index[i]].y * edge_point_nor[rand_index[i]].y;
			A.at<double>(i, 3) = edge_point_nor[rand_index[i]].x;
			A.at<double>(i, 4) = edge_point_nor[rand_index[i]].y;
		}

		SVD::compute(A, S, U, V);

		for (unsigned i = 0; i < 6; i++)
			pd[i] = S.at<double>(i);

		min_d_index = 0;
		for (unsigned i = 1; i < 6; i++)
		{
			if (pd[i] < pd[min_d_index])
				min_d_index = i;
		}

		for (unsigned i = 0; i < 6; i++)
		{
			//the column of v that corresponds to the smallest singular value, which is the solution of the equations
			conic_parameters[i] = V.at<double>(min_d_index, i);
		}

		ninliers = 0;
		memset(inliers_index, 0, sizeof(int)*edge_point.size());
		for (unsigned i = 0; i < edge_point.size(); i++)
		{
			// ellipse equation with coefficients given by conic parameters 
			// dis_error goes to 0 at the approach of the point on the ellispe
			dis_error = conic_parameters[0] * edge_point_nor[i].x*edge_point_nor[i].x +
				conic_parameters[1] * edge_point_nor[i].x*edge_point_nor[i].y +
				conic_parameters[2] * edge_point_nor[i].y*edge_point_nor[i].y +
				conic_parameters[3] * edge_point_nor[i].x + conic_parameters[4] * edge_point_nor[i].y + conic_parameters[5];
			
			// if the point is near enough it is an inlier
			if (fabs(dis_error) < dis_threshold)
			{
				inliers_index[ninliers] = i;
				ninliers++;
			}
		}

		if (ninliers > max_inliers)
		{
			if (SolveEllipse(conic_parameters, ellipse_parameters))
			{
				DenormalizeEllipseParameters(ellipse_parameters, ellipse_parameters, dis_scale, nor_center);

				// ratio between ellipse axes
				ratio = ellipse_parameters[0] / ellipse_parameters[1]; 
	
				if (ellipse_parameters[2] > 0 && ellipse_parameters[2] <= width - 1 && ellipse_parameters[3] > 0 &&
					ellipse_parameters[3] <= height - 1 && ratio > 0.5 && ratio < 2)
				{
					memcpy(max_inliers_index, inliers_index, sizeof(int)*edge_point.size());
					for (unsigned i = 0; i < 5; i++)
						best_ellipse_parameters[i] = ellipse_parameters[i];

					max_inliers = ninliers;
					sample_num = (int)(log((double)(1 - 0.99)) / log(1.0 - pow((double)ninliers / edge_point.size(), 5)));
				}
			}
		}
		ransac_count++;
		
		if (ransac_count > 1500) 
		{
			cout << "Error! ransac_count exceed! ransac break! sample_num=" << sample_num << " , ransac_count=" << ransac_count << endl;
			break;
		}
	}

	if (best_ellipse_parameters[0] > 0 && best_ellipse_parameters[1] > 0)
	{
		for (unsigned i = 0; i < 5; i++)
			pupil_parameters[i] = best_ellipse_parameters[i];
	}
	else
	{
		pupil_parameters.assign(pupil_parameters.size(), 0);
		max_inliers = 0;
		std::free(max_inliers_index);
		max_inliers_index = NULL;
	}

	std::free(edge_point_nor);
	std::free(inliers_index);
	return_max_inliers = max_inliers;
	return max_inliers_index;
}