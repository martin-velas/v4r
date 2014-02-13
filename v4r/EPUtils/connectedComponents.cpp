#include "connectedComponents.hpp"

namespace EPUtils
{

ConnectedComponent::ConnectedComponent()
{
  points.clear();
  saliency_values.clear();
  average_saliency = 0;
}

void extractConnectedComponents(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th)
{
  assert(map.type() == CV_32FC1);
  assert((th >= 0) && (th <= 1));
  
  cv::Mat map_copy;
  map.copyTo(map_copy);
  for(int i = 0; i < map_copy.rows; ++i)
  {
    for(int j = 0; j < map_copy.cols; ++j)
    {
      if(map_copy.at<float>(i,j) > th)
      {
	ConnectedComponent new_component;
	
	new_component.points.push_back(cv::Point(j,i));
	new_component.saliency_values.push_back(map_copy.at<float>(i,j));
	new_component.average_saliency = map_copy.at<float>(i,j);
	
	map_copy.at<float>(i,j) = 0;
	
	std::vector<cv::Point> queue;
	queue.push_back(cv::Point(j,i));
	
	while(queue.size())
	{
	  cv::Point cur_point = queue.back();
	  queue.pop_back();
	  
	  for(int p = 0; p < 8; ++p)
          {
            int new_x = cur_point.x + dx8[p];
            int new_y = cur_point.y + dy8[p];
	    
	    if((new_x < 0) || (new_y < 0) || (new_x >= map_copy.cols) || (new_y >= map_copy.rows))
	      continue;
	    
	    if(map_copy.at<float>(new_y,new_x) > th)
            {
	      new_component.points.push_back(cv::Point(new_x,new_y));
	      new_component.saliency_values.push_back(map_copy.at<float>(new_y,new_x));
	      new_component.average_saliency += map_copy.at<float>(new_y,new_x);
	      
	      map_copy.at<float>(new_y,new_x) = 0;
	      
	      queue.push_back(cv::Point(new_x,new_y));
	    }
	  }
	}
	
	new_component.average_saliency /= new_component.points.size();
	
	if(new_component.average_saliency > th)
	{
	  connected_components.push_back(new_component);
	}
	
      }
    }
  }
}

/*void extractConnectedComponents2(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th)
{
  assert(map.type() == CV_32FC1);
  assert((th >= 0) && (th <= 1));
  
  cv::Mat map_copy;
  map.copyTo(map_copy);
  
  double maxVal=0;
  cv::Point maxLoc;
  cv::minMaxLoc(map,0,&maxVal,0,&maxLoc);
  
  while(maxVal > 0)
  {
    //if(count>20)
      cv::Point maxLoc_new;
      winnerToImgCoords(maxLoc_new,maxLoc,mapLevel);
      centers.push_back(maxLoc_new);
    //count = 1;
    
    float maxValTh = (1-th)*maxVal;
    
    std::list<cv::Point> points;
    points.push_back(maxLoc);
    cv::Mat used = cv::Mat_<uchar>::zeros(map.rows,map.cols);
    used.at<uchar>(maxLoc.y,maxLoc.x) = 1;
    map.at<float>(maxLoc.y,maxLoc.x) = 0;
    while(points.size())
    {
      cv::Point p = points.front();
      points.pop_front();
      
      if(((p.x+1) < map.cols) && (!used.at<uchar>(p.y,p.x+1)) && (map.at<float>(p.y,p.x+1)>maxValTh))
      {
        points.push_back(cv::Point(p.x+1,p.y));
        used.at<uchar>(p.y,p.x+1) = 1;
	map.at<float>(p.y,p.x+1) = 0;
	//count++;
      }
      if(((p.x-1) >= 0) && (!used.at<uchar>(p.y,p.x-1)) && (map.at<float>(p.y,p.x-1)>maxValTh))
      {
        points.push_back(cv::Point(p.x-1,p.y));
        used.at<uchar>(p.y,p.x-1) = 1;
	map.at<float>(p.y,p.x-1) = 0;
	//count++;
      }
      if(((p.y+1) < map.rows) && (!used.at<uchar>(p.y+1,p.x)) && (map.at<float>(p.y+1,p.x)>maxValTh))
      {
        points.push_back(cv::Point(p.x,p.y+1));
        used.at<uchar>(p.y+1,p.x) = 1;
	map.at<float>(p.y+1,p.x) = 0;
	//count++;
      }
      if(((p.y-1) >= 0) && (!used.at<uchar>(p.y-1,p.x)) && (map.at<float>(p.y-1,p.x)>maxValTh))
      {
        points.push_back(cv::Point(p.x,p.y-1));
        used.at<uchar>(p.y-1,p.x) = 1;
	map.at<float>(p.y-1,p.x) = 0;
	//count++;
      }
    }
    cv::minMaxLoc(map,0,&maxVal,0,&maxLoc);
    
    //cv::imshow("map",map);
    //cv::waitKey();
  }
}*/

void drawConnectedComponent(ConnectedComponent component, cv::Mat &image, cv::Scalar color)
{
  int nchannels = image.channels();
  for(unsigned j = 0; j < component.points.size(); ++j)
  {
    
    int x = component.points.at(j).x;
    int y = component.points.at(j).y;
    
    for(int i = 0; i < nchannels; ++i)
    {
      image.at<uchar>(y,nchannels*x+i) = color(i);
    }
  }
}

void drawConnectedComponents(std::vector<ConnectedComponent> components, cv::Mat &image, cv::Scalar color)
{
  for(unsigned int i = 0; i < components.size(); ++i)
  {
    drawConnectedComponent(components.at(i),image,color);
  }
}

} //namespace EPUtils