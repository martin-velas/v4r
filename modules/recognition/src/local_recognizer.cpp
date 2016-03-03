#include <v4r/recognition/impl/local_recognizer.hpp>
#include <v4r/common/faat_3d_rec_framework_defines.h>

//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> >;
template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<352> >;
template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<128> >;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<1344> >;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33>;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::FPFHSignature33>;

//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::Histogram<352> >;
template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<128> >;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<352> >;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<1344> >;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::FPFHSignature33>;
//template class V4R_EXPORTS v4r::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::FPFHSignature33>;
