#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <queue>
using namespace cv;
using namespace std;
class Pixel;
class Edge {
public:
	double edgeWeight;
	int secondpixel;
	Edge(double edge, int second)
	{
		edgeWeight = edge;
		secondpixel = second;
	}
};
class Pixel {
public:
	int pnumber;
	double edgeWeight;
	bool visited;
	int front;
	int back;
	vector<Edge> edges;
	Pixel()
	{
		pnumber = INT_MIN;
		visited = false;
		front = back = INT_MIN;
		edgeWeight = LONG_MAX;
	}
	void addEdge(double w, int second)
	{
		Edge edg(w, second);
		edges.push_back(edg);
	}
	void setPixelNumber(int number)
	{
		pnumber = number;
	}
};
bool checkForPathiSourceToiSinkBFS(vector<Pixel> &pixels, Pixel &imaginarySource, Pixel &imaginarySink)
{
	queue<Pixel> q;
	q.push(imaginarySource);
	imaginarySource.visited = true;
	int secondpixelIndex;
	while (!q.empty())
	{
		Pixel currentPixel = q.front();
		q.pop();
		for (int i = 0; i < currentPixel.edges.size(); i++)
		{
			double edgeWeight = currentPixel.edges[i].edgeWeight;
			secondpixelIndex = currentPixel.edges[i].secondpixel;
			if (secondpixelIndex < pixels.size() - 2)
			{
				Pixel &nextpixel = pixels.at(secondpixelIndex);
				if (!nextpixel.visited && edgeWeight > 0)
				{
					nextpixel.visited = true;
					nextpixel.edgeWeight = edgeWeight;
					nextpixel.back = currentPixel.pnumber;
					q.push(nextpixel);
				}
			}
			else if (secondpixelIndex == pixels.size() - 1 && edgeWeight > 0)
			{
				imaginarySink.visited = true;
				imaginarySink.edgeWeight = LONG_MAX;
				imaginarySink.back = currentPixel.pnumber;
				return true;
			}
		}
	}
	return false;
}
double getMINflow(vector<Pixel> &pixels, Pixel source, Pixel sink)
{
	double min_flow = LONG_MAX;
	int start = sink.pnumber;
	while (start != source.pnumber)
	{
		if (min_flow > pixels[start].edgeWeight)
			min_flow = pixels[start].edgeWeight;
		int tmp = start;
		start = pixels[start].back;
		pixels[start].front = tmp;
	}
	return min_flow;
}
void updateRESIDUALgraph(vector<Pixel> &pixels, Pixel source, Pixel sink, double path_flow)
{
	int start = source.pnumber;
	while (start != sink.pnumber)
	{
		for (int i = 0; i < pixels[start].edges.size(); i++)
		{
			if (pixels[start].edges[i].secondpixel == pixels[start].front)
			{
				pixels[start].edges[i].edgeWeight = pixels[start].edges[i].edgeWeight - path_flow;
				break;
			}
		}
		int next = pixels[start].front;
		for (int i = 0; i < pixels[next].edges.size(); i++)
		{
			if (pixels[next].edges[i].secondpixel == start)
			{
				pixels[next].edges[i].edgeWeight = pixels[next].edges[i].edgeWeight + path_flow;
				break;
			}
		}
		start = pixels[start].front;
	}
}
void resetVisitedPixels(vector<Pixel> &pixels)
{
	for (int i = 0; i < pixels.size(); i++)
	     pixels[i].visited = false;
}
void computeMINcutMAXflow(vector<Pixel> &pixels, Mat &out_image,int width)
{
        double minflow = LONG_MAX;
	while (checkForPathiSourceToiSinkBFS(pixels, pixels[pixels.size() - 2], pixels[pixels.size() - 1])) 
        {
  	        resetVisitedPixels(pixels);
	        minflow = getMINflow(pixels, pixels[pixels.size() - 2], pixels[pixels.size() - 1]);
     		updateRESIDUALgraph(pixels, pixels[pixels.size() - 2], pixels[pixels.size() - 1], minflow);
	}
	Vec3b background;
	background[0] = 0;
	background[1] = 0;
	background[2] = 0;
	Vec3b foreground;
	foreground[0] = 255;
	foreground[1] = 255;
	foreground[2] = 255;
	for (int i = 0; i < pixels.size() - 2; i++)
	{
		int  row = pixels[i].pnumber / width;
		int col = pixels[i].pnumber % width;
		if (!pixels[i].visited)
			out_image.at<Vec3b>(row, col) = background;
		else
			out_image.at<Vec3b>(row, col) = foreground;
	}
}
int main(int argc, char** argv)
{
	if (argc != 4) {
		cout << "Usage: ../seg input_image initialization_file output_mask" << endl;
		return -1;
	}   
	Mat in_image, gaussian_blur_image, gray_image;
	in_image = imread(argv[1]);
	if (!in_image.data)
	{
		cout << "Could not load input image!!!" << endl;
		return -1;
	}
	if (in_image.channels() != 3) {
		cout << "Image does not have 3 channels!!!3 " << in_image.depth() << endl;
		return -1;
	}
	GaussianBlur(in_image, gaussian_blur_image, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(gaussian_blur_image, gray_image, COLOR_BGR2GRAY);
	Mat out_image = in_image.clone();
	ifstream f(argv[2]);
	if (!f) {
		cout << "Could not load initial mask file!!!" << endl;
		return -1;
	}
	double limit = 1;
	int width = in_image.cols;
	int height = in_image.rows;
	int pixelsCount = width * height;
	vector<Pixel> allpixels(pixelsCount + 2);
	int n;
	f >> n;
	for (int i = 0; i < allpixels.size(); i++)
		allpixels[i].setPixelNumber(i);
	int sourceAddress = pixelsCount;
	int sinkAddress = pixelsCount + 1;
	for (int i = 0; i < n; ++i)
	{
		int x, y, t;
		f >> x >> y >> t;
		if (x < 0 || x >= width || y < 0 || y >= height) {
			cout << "Invalid pixel mask!" << endl;
			return -1;
		}
		if (t == 1) 
			allpixels[sourceAddress].addEdge(LONG_MAX, y*width + x);
		else if (t == 0) 
			allpixels[y*width + x].addEdge(LONG_MAX, sinkAddress);
	}
	double difference;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (0 <= i - 1)
			{
				difference = gray_image.at<uchar>(i, j) - gray_image.at<uchar>(i - 1, j);
				if (difference < limit)
					allpixels[i*width + j].addEdge(LONG_MAX, (i - 1)*width + j);
				else
					allpixels[i*width + j].addEdge(1.0, (i - 1)*width + j);
			}
			if (i + 1 < height)
			{
				difference = gray_image.at<uchar>(i, j) - gray_image.at<uchar>(i + 1, j);
				if (difference < limit)
					allpixels[i*width + j].addEdge(LONG_MAX, (i + 1)*width + j);
				else
					allpixels[i*width + j].addEdge(1.0, (i + 1)*width + j);
			}
			if (0 <= j - 1)
			{
				difference = gray_image.at<uchar>(i, j) - gray_image.at<uchar>(i, j - 1);
				if (difference < limit)
					allpixels[i*width + j].addEdge(LONG_MAX, i*width + j - 1);
				else
					allpixels[i*width + j].addEdge(1.0, i*width + j - 1);
			}
			if (j + 1 < width)
			{
				difference = gray_image.at<uchar>(i, j) - gray_image.at<uchar>(i, j + 1);
				if (difference < limit)
					allpixels[i*width + j].addEdge(LONG_MAX, i*width + j + 1);
				else
					allpixels[i*width + j].addEdge(1.0, i*width + j + 1);
			}
		}
	}
	computeMINcutMAXflow(allpixels, out_image, width);
	namedWindow("Original image", WINDOW_AUTOSIZE);
	namedWindow("Show Marked Pixels", WINDOW_AUTOSIZE);
	imshow("Original image", in_image);
	imshow("Show Marked Pixels", out_image);
	imwrite(argv[3], out_image);
	waitKey(0);
	return 0;
}
