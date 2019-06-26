#include "corneal_reflection.h"

int crar;

/** remove corneal reflection
 * @param src: source image
 * @param dest: destination image
 * @param sp: start point coordinates
 * @param window_size: dimesnion of ROI
 * @param biggest_crr: maximum radius of corneal reflection
 * @param cr: corneal reflection coordinates
 * @param crr: corneal reflection radius
*/
void RemoveCornealReflection(ofxCvGrayscaleImage& src, ofxCvGrayscaleImage& dest, Point sp, int window_size,
	int biggest_crr, Point& cr, int& crr)
{
	// initialize points
	crar = -1;
	cr.x = cr.y = crr = -1;

	float angle_delta = PI / 180;
	int angle_num = (int)(2 * PI / angle_delta);
	vector<double> angle_array(angle_num);
	vector<double> sin_array(angle_num);
	vector<double> cos_array(angle_num);

	for (unsigned i = 0; i < angle_num; i++)
	{
		angle_array[i] = i * angle_delta;
		sin_array[i] = sin(angle_array[i]);
		cos_array[i] = cos(angle_array[i]);
	}

	// locate corneal reflection
	LocateCornealReflection(src, dest, sp, window_size, (int)(biggest_crr/2.5), cr, crr);

	// corneal reflection radius calculation
	crr = FitCircleRadiusToCornealReflection(src, cr, crar, (int)(biggest_crr / 2.5), sin_array, cos_array);
	crr = (int)(2.5 * crr);

	// radial interpolation to remove effectively corneal reflection
	InterpolateCornealReflection(src, cr, crr, sin_array, cos_array);

	angle_array.clear();
	cos_array.clear();
	sin_array.clear();
}

/** remove corneal reflection
* @param src: source image
* @param dest: destination image
* @param sp: start point coordinates
* @param window_size: dimesnion of ROI
* @param biggest_crr: maximum radius of corneal reflection
* @param cr: corneal reflection coordinates 
* @param crr: corneal reflection radius
*/
void LocateCornealReflection(ofxCvGrayscaleImage& src, ofxCvGrayscaleImage& dest, Point sp, int window_size,
	int biggest_crar, Point& cr, int& crr)
{
	int r = (window_size - 1) / 2;
	// center ROI window on start point
	int startx = MAX(sp.x - r, 0);
	int endx = MIN(sp.x + r, src.getCvImage()->width - 1);
	int starty = MAX(sp.y - r, 0);
	int endy = MIN(sp.y + r, src.getCvImage()->height - 1);

	// set ROI
	src.setROI(startx, starty, endx - startx + 1, endy - starty + 1);
	dest.setROI(startx, starty, endx - startx + 1, endy - starty + 1);

	// determine minimum and maximum value of pixels' intensity in source image
	double min_value, max_value;
	cvMinMaxLoc(src.getCvImage(), &min_value, &max_value); 
	
	CvSeq* contour = NULL;
	CvMemStorage* storage = cvCreateMemStorage();
	double *scores = (double*)malloc(sizeof(double)*((int)max_value + 1));
	memset(scores, 0, sizeof(double)*((int)max_value + 1));
	int threshold;
	double area, max_area, sum_area;
	int num_areas;

	for (threshold = (int)max_value; threshold >= 1; threshold--)
	{
		// apply threshold to obtain a binary image
		cvThreshold(src.getCvImage(), dest.getCvImage(), threshold, 1, CV_THRESH_BINARY);
		// contour detection on binary image 
		cvFindContours(dest.getCvImage(), storage, &contour);
		max_area = 0;
		sum_area = 0;
		num_areas = 0;
		CvSeq *max_contour = contour;
		for (; contour != 0; contour = contour->h_next)
		{
			// calculate number of pixels within countour just detected
			area = contour->total + (int)(fabs(cvContourArea(contour, CV_WHOLE_SEQ)));
			num_areas++;
			sum_area += area;
			if (area > max_area) 
			{
				max_area = area;
				max_contour = contour;
			}
		}

		if (sum_area - max_area > 0)
			scores[threshold - 1] = max_area / (sum_area - max_area) * num_areas;   
		else
			continue;

		// take the threshold that generates the highest ratio between area of maximum contour and area of other contours
		if (scores[threshold - 1] - scores[threshold] < 0) 
		{
			crar = (int)sqrt(max_area / PI); 
			int sum_x = 0;
			int sum_y = 0;
			CvPoint *point;

			// slide every point of maximum contour
			for (unsigned i = 0; i < max_contour->total; i++) 
			{
				point = CV_GET_SEQ_ELEM(CvPoint, max_contour, i);
				sum_x += point->x;
				sum_y += point->y;
			}

			cr.x = sum_x / max_contour->total;
			cr.y = sum_y / max_contour->total;
			break;
		}
	}

	free(scores);
	cvReleaseMemStorage(&storage);
	
	// reset ROI
	src.resetROI();
	dest.resetROI();

	if (crar > biggest_crar) 
	{
		cout << "corneal size wrong! " << endl;
		cr.y = cr.x = -1;
		crar = -1;
	}

	if (cr.x != -1 && cr.y != -1)
	{
		/*cout << "(corneal) startx: " << startx << ", starty: " << starty << " cr: (" 
			<< cr.x << ", " << cr.y << "), crr = " << crr << endl;*/
		cr.x += startx;
		cr.y += starty;
	}
}

/** calculate radius for corneal reflection
 * @param src: source image
 * @param dest: destination image
 * @param cr: corneal reflection coordinates
 * @param crar: corneal reflection approximate radius
 * @param biggest_crar: maximum corneal reflection approximate radius
 * @param sin_array: array of sin values along the angles
 * @param cos_array: array of cos values along the angles
 * @return corneal reflection approximate radius
*/
int FitCircleRadiusToCornealReflection(ofxCvGrayscaleImage& src, const Point& cr, int crar, int biggest_crar,
	const vector<double>& sin_array, const vector<double>& cos_array)
{
	if (cr.x == -1 || cr.y == -1 || crar == -1)
		return -1;

	// find r through a Nelder-Mead Simplex that minimizes the ratio between the integrals
	vector<double> ratio(biggest_crar - crar + 1);
	int r, r_delta = 1;
	int x_num, y_num, x_den, y_den;
	double sum_num, sum_den;

	for (r = crar; r <= biggest_crar; r++) 
	{
		sum_num = 0;
		sum_den = 0;

		for (unsigned i = 0; i < cos_array.size(); i++)
		{
			x_num = (int)(cr.x + (r + r_delta)*cos_array[i]);
			y_num = (int)(cr.y + (r + r_delta)*sin_array[i]);
			x_den = (int)(cr.x + (r - r_delta)*cos_array[i]);
			y_den = (int)(cr.y + (r - r_delta)*sin_array[i]);
			if ((x_num >= 0 && y_num >= 0 && x_num < src.getWidth() && y_num < src.getHeight()) &&
				(x_den >= 0 && y_den >= 0 && x_den < src.getWidth() && y_den < src.getHeight()))
			{
				// integral of pixels' intensity calculated over a circle with (r+r_delta) radius and cr center
				sum_num += *(src.getCvImage()->imageData + y_num * src.getCvImage()->width + x_num);
				// integral of pixels' intensity calculated over a circle with (r-r_delta) radius and cr center
				sum_den += *(src.getCvImage()->imageData + y_den * src.getCvImage()->width + x_den);
			}
		}

		// ratio between the previous integrals
		ratio[r - crar] = sum_num / sum_den;

		if (r - crar >= 2) 
		{
			// minimize the ratio
			if (ratio[r - crar - 2] > ratio[r - crar - 1] && ratio[r - crar] > ratio[r - crar - 1]) 
			{
				ratio.clear();

				return r - 1;
			}
		}
	}

	ratio.clear();

	cout << "ATTN! corneal reflection radius not changed" << endl;

	return crar;
}

/** effectively remove corneal reflection from the image by interpolating pixels' intensity 
* @param src: source image
* @param cr: corneal reflection coordinates
* @param crr: corneal reflection radius
* @param sin_array: array of sin values along the angles
* @param cos_array: array of cos values along the angles
*/
void InterpolateCornealReflection(ofxCvGrayscaleImage& src, const Point& cr, int crr, const vector<double>& sin_array,
	const vector<double>& cos_array)
{
	if (cr.x == -1 || cr.y == -1 || crr == -1)
		return;

	if (cr.x - crr < 0 || cr.x + crr >= src.getCvImage()->width || 
		cr.y - crr < 0 || cr.y + crr >= src.getCvImage()->height) 
	{
		cout << "Error! Corneal reflection is too near the image border" << endl;
		return;
	}

	int r, r_complementare, x, y;

	vector<unsigned char> perimeter_pixel(cos_array.size());
	int sum = 0;

	for (unsigned i = 0; i < cos_array.size(); i++)
	{
		// circumference with cr center and crr radius
		x = (int)(cr.x + crr * cos_array[i]);
		y = (int)(cr.y + crr * sin_array[i]);

		// fill vector with pixel of corneal reflection contour
		perimeter_pixel[i] = *(src.getCvImage()->imageData + y * src.getCvImage()->width + x);
		sum += perimeter_pixel[i];
	}

	// average value of contour pixel
	double avg = (double)sum / cos_array.size();

	// coloring corneal reflection by interpolation
	for (r = 1; r < crr; r++)
	{
		r_complementare = crr - r;
		
		for (unsigned i = 0; i < cos_array.size(); i++)
		{
			// circumference with cr center and r radius
			x = (int)(cr.x + r * cos_array[i]);
			y = (int)(cr.y + r * sin_array[i]);

			// weighted sum in the case of partial overlap of corneal reflection and pupil
			*(src.getCvImage()->imageData + y * src.getCvImage()->width + x) = 
				(unsigned char)(((double)r_complementare / crr) * avg + ((double)r / crr) * perimeter_pixel[i]);
		}
	}

	perimeter_pixel.clear();
}
