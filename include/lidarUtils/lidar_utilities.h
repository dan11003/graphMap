#ifndef LIDAR_UTILITIES_H
#define LIDAR_UTILITIES_H

#include <laser_geometry/laser_geometry.h>
#include "gnuplot-iostream.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "strings.h"
using namespace std;
namespace libgraphMap {



class ScanPlot{
public:
  typedef enum PlotType{rangeAnglePlot=0,
                        xyPlot        =1,
                       multiPlotRangeAngleXY=2,
                       rangeAnglePlotWithDerivative=3}PlotType;

  void plotScan(sensor_msgs::LaserScan &scan,const PlotType plot_type=rangeAnglePlot,const string PlotOptsX="[-20:20]",const string PlotOptsY="[-20:20]");
private:
  void plotScanRangeAngle(sensor_msgs::LaserScan &scan,const string PlotOptsX,string PlotOptsY);
  void plotScanXY(sensor_msgs::LaserScan &scan,const string PlotOptsX,string PlotOptsY);
  void derivative(sensor_msgs::LaserScan &scan, std::vector<std::pair<double, double> > &derivative);
  Gnuplot gp;


};
}
#endif // LIDAR_UTILITIES_H
