#include "lidarUtils/lidar_utilities.h"

namespace libgraphMap {
void ScanPlot::plotScanXY(sensor_msgs::LaserScan &scan,const string PlotOptsX,string PlotOptsY){

  std::vector<std::pair<double, double> > dataSet;
  for(int i=0;i<scan.ranges.size();i++){
    double angle=scan.angle_min+(double)i/(double)(scan.ranges.size())*(double)(scan.angle_max-scan.angle_min);
    double range=scan.ranges[i];
    double x=range*cos(angle+3.1415/2);
    double y=range*sin(angle+3.1415/2);
    dataSet.push_back(std::make_pair(x,y));
  }
  gp  <<"unset autoscale"<<endl;
  gp << "set xrange "<<PlotOptsX<<"\nset yrange "<<PlotOptsY<<"\n"<<endl;
  gp << "plot" << gp.file1d(dataSet) << "with points title 'cartesian'"<<endl;

}
void ScanPlot::plotScanRangeAngle(sensor_msgs::LaserScan &scan,const string PlotOptsX,string PlotOptsY){

  std::vector<std::pair<double, double> > dataSet;
  for(int i=0;i<scan.ranges.size();i--){
    double angle=(scan.angle_min+(double)i/(double)(scan.ranges.size())*(double)(scan.angle_max-scan.angle_min));
    double range=scan.ranges[i];
    dataSet.push_back(std::make_pair(-angle,range ));
  }
  gp <<"unset autoscale"<<endl;
  gp << "set xrange "<<PlotOptsX<<"\nset yrange "<<PlotOptsY<<"\n"<<endl;;
  gp << "plot" << gp.file1d(dataSet) << "with points title 'range angle'"<<endl;

}


void ScanPlot::derivative(sensor_msgs::LaserScan &scan,std::vector<std::pair<double, double> > &der){
  der.clear();
  for(int i=1;i<scan.ranges.size()-1;i++){
  double derivative=(1/2*(scan.ranges[i+1])-scan.ranges[i-1]);
  double angle=scan.angle_min+scan.angle_increment+(double)i/(double)(scan.ranges.size())*(double)(scan.angle_max-scan.angle_min);
  der.push_back(std::make_pair(angle,derivative));
  }


 /* cout<<"range size="<<scan.ranges.size()<<", derivative vector size="<<der.size()<<endl;
  for(int i=0;i<der.size();i++)
    cout<<der[i]<<",";

  cout<<"\n ....end of poits"<<endl; */
}


void ScanPlot::plotScan(sensor_msgs::LaserScan &scan,const PlotType plot_type,const string PlotOptsX,const string PlotOptsY){

  switch(plot_type){

  case rangeAnglePlot:
    plotScanRangeAngle(scan,PlotOptsX,PlotOptsY);
    break;
  case xyPlot:
    plotScanXY(scan,PlotOptsX,PlotOptsY);
    break;
  case multiPlotRangeAngleXY:
    gp<<"set multiplot layout 2,2 columnsfirst margins 0.1,0.9,0.1,0.9 spacing 0.1"<<endl;
    plotScanRangeAngle(scan,PlotOptsX,PlotOptsY);
    plotScanXY(scan,PlotOptsX,PlotOptsY);
    gp<<"unset multiplot"<<endl;
    break;
  case rangeAnglePlotWithDerivative:
    gp<<"set multiplot layout 2,2 columnsfirst margins 0.1,0.9,0.1,0.9 spacing 0.1"<<endl;
    plotScanRangeAngle(scan,PlotOptsX,PlotOptsY);
    std::vector<std::pair<double, double> > der;
    derivative(scan,der);
    gp <<"unset autoscale"<<endl;
    gp << "set xrange "<<PlotOptsX<<"\nset yrange "<<PlotOptsY<<"\n"<<endl;;
    gp << "plot" << gp.file1d(der) << "with points title 'range angle'"<<endl;
    cout<<"der size"<<der.size()<<endl;
    gp<<"unset multiplot"<<endl;
    break;

  }


}
}
