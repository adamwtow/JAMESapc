#include <capsicum_detector.h>

int threshh_probability_HSV = 40;
int image_number = 0;

cv::RNG rng(12345);


////////////////Constructors

capsicum::capsicum(cv::Moments mu, cv::Rect boundRect)
{
    area = mu.m00;
    center_point = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    moments = mu;					//m00, m01, m10
    bounding_box = boundRect;
}

capsicum_detector::capsicum_detector(HSV_model model)
{
    capsicum_model = model;
}

capsicum_detector::capsicum_detector(){

}

cv::Mat capsicum_detector::detect(cv::Mat &color_image, cv::Mat &depth, int minArea){

    cv::Mat contour_image, depth_mask, depth_color;
    cv::Mat segmentedImage, filtered_image, masked_image;
    //create depth mask and mask color image
    inRange(depth,cv::Scalar(500),cv::Scalar(1500),depth_mask);
    color_image.copyTo(depth_color,depth_mask);

    //cv::imshow("Colour",depth_color);
    //cv::waitKey(10);

    //open_image(&image, files);
    filter_image(depth_color, filtered_image);
    //segmentImage(depth_color, segmentedImage,0.0);
    segmentImage(capsicum_model, depth_color, segmentedImage);

	segmentedImage.copyTo(contour_image);

	findContours( contour_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );


    std::vector<std::vector<cv::Point> > capsicum_contours;
    std::vector<cv::Rect> boundRect_temp;

    std::vector<cv::Point2f>center_temp;
    std::vector<float>radius_temp;
    std::vector<cv::Moments> mu_temp;

    masked_image = cv::Mat::zeros( depth_color.rows, depth_color.cols, CV_8UC3 );
    cv::Mat bounding_box_mask = cv::Mat::zeros( depth_color.rows, depth_color.cols, CV_8UC3 );

    nCapsicumFound = 0;

	for( int j = 0; j < contours.size(); j++ )
    {
        if(cv::contourArea(contours[j]) > minArea)
        {
            nCapsicumFound++;

            mu_temp.push_back(cv::moments( contours[j], false ));

            // Approximate contours to polygons + get bounding rects and circles
            std::vector<cv::Point> approx_contour;
            approxPolyDP( cv::Mat(contours[j]), approx_contour, 3, true );
            capsicum_contours.push_back(approx_contour);

            //find center point of contour
            cv::Point2f center;
            float radius;
            minEnclosingCircle( (cv::Mat)approx_contour, center, radius );
            center_temp.push_back(center);
            radius_temp.push_back(radius);

            //draw bounding box and center point
            cv::Scalar color = cv::Scalar(  255, 255, 0 );

            cv::Rect boundRect = boundingRect( cv::Mat(approx_contour) );
            boundRect_temp.push_back(boundRect);
            rectangle( bounding_box_mask , boundRect.tl(), boundRect.br(), color, 4, 8, 0 );
            circle( bounding_box_mask, center, 2, color, 2, 8, 0 );
        }
    }

    mu = mu_temp;
    contours = capsicum_contours;
    boundRect = boundRect_temp;
    center = center_temp;
    radius =  radius_temp;

    addWeighted(color_image, 1, bounding_box_mask , 1, 0.0, masked_image);

    return masked_image;
}




double capsicum_detector::LikelihoodOfNormal(double mean, double variance, double pixel_value)
{
	//do some math
    double A = 1./(std::sqrt(M_PI*2.*variance));
	double B = std::pow((pixel_value-mean), 2.);
    double C = 2*variance;

	double out_put_pixel = A*exp(-B/C);
	return out_put_pixel;
}


void capsicum_detector::segmentImage(cv::Mat &inputImage, cv::Mat &outputImage, double threshold)
{
    //double minVal;
    //double maxVal;
    cv::Mat splitImage[3];
    cv::Mat thresholdImage;

    double output_pixel;

    //split image into 3 color channels
    cv::split(inputImage, splitImage);

    //allocates memory for output_image
    outputImage.create(splitImage[0].rows, splitImage[0].cols, CV_64F);

    //2*red - 2*green
    outputImage = 2*splitImage[2] - 2*splitImage[1] - splitImage[0];

    outputImage.convertTo(outputImage, CV_8U);

    thresholdImage = outputImage;

    //threshold image
    cv::threshold(thresholdImage, outputImage, threshold, 255, 0);

}

void capsicum_detector::segmentImage(HSV_model A, cv::Mat &inputImage, cv::Mat &outputImage)
{
    //double minVal;
    //double maxVal;
    cv::Mat splitImage[3];
    cv::Mat thresholdImage, gray_image, HSV_Image;

    double channel_0;
    double channel_1;
    double output_pixel;

    cvtColor(inputImage, HSV_Image, CV_BGR2HSV );

    //split image into 3 color channels
    cv::split(HSV_Image, splitImage);

    //allocates memory for output_image
    outputImage.create(splitImage[0].rows, splitImage[0].cols, CV_64F);

    //Performs the likelihood of normal function on each pixel
    for(int i = 0; i < splitImage[0].rows; i++)
    {
        for(int j = 0; j < splitImage[0].cols; j++)
            {
                uint16_t h_temp = splitImage[0].at<unsigned char>(i,j);
                h_temp += 90;
                splitImage[0].at<unsigned char>(i,j) = (uint8_t)(h_temp%179);


                channel_0 = (double)splitImage[0].at<unsigned char>(i,j);
                channel_1 = (double)splitImage[1].at<unsigned char>(i,j);

                channel_0 = LikelihoodOfNormal(A.hue_mean, A.hue_var, channel_0);
                channel_1 = LikelihoodOfNormal(A.saturation_mean, A.saturation_var, channel_1);


                output_pixel = channel_0*channel_1/(4.4*pow(10,-4.));
                //z = h

                outputImage.at<double>(i,j) = output_pixel;
            }
    }

    ////////////////Finds the max and min values of a Mat
    //cv::minMaxLoc(*outputImage, &minVal, &maxVal);
    //std::cout << maxVal << std::endl;

    outputImage = outputImage*255.;

    outputImage.convertTo(outputImage, CV_8U);

    thresholdImage = outputImage;

    //threshold image
    cv::threshold(thresholdImage, outputImage, threshh_probability_HSV, 255, 0);
}

//////////opens the next valid image form the list files
int capsicum_detector::open_image(cv::Mat &Image, std::vector<std::string> &files)
{
    int no_image = 0;
    if(image_number >= files.size())
    {
        std::cout <<  "All files opened" << std::endl ;
        return -1;
    }

    while(no_image == 0)
    {
        Image = cv::imread("/home/chris/Pictures/farm/"+files[image_number], CV_LOAD_IMAGE_COLOR);   // Read the file

        if(! Image.data )                              // Check for invalid input
        {
            std::cout <<  "Could not open or find the image" << std::endl ;
            image_number++;
            //return -1;
        }
        else
        {
            std::cout <<  "Image Opened "+files[image_number] << std::endl ;
            no_image ++;
        }
        image_number++;
    }

    return 1;

}

void capsicum_detector::filter_image(cv::Mat &Image, cv::Mat &filtered_Image)
{
    int MAX_KERNEL_LENGTH = 25;

    cv::blur(Image, filtered_Image, cv::Size(MAX_KERNEL_LENGTH,MAX_KERNEL_LENGTH), cv::Point(-1,-1));

}

