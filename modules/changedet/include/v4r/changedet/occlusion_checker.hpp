#ifndef __OCCLUSION_CHECKER__H
#define __OCCLUSION_CHECKER__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/segment_differences.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <tf/tf.h>

namespace v4r {

template <class PointType>
class OcclusionChecker {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    struct occluded_points
    {
        CloudPtr toBeAdded;
        CloudPtr toBeRemoved;
    };

    struct occlusion_results {
        CloudPtr occluded;
        CloudPtr nonOccluded;
    };

    OcclusionChecker() :
    	viewpoint(Eigen::Vector3f(0.0,0.0,0.0)),
    	numberOfBins(360),
    	tolerance(0.001) {
    }

    void setViewpoint(Eigen::Vector3f origin)
    {
        viewpoint = origin;
    }

    void setViewpoint(tf::Vector3 origin)
    {
        viewpoint = Eigen::Vector3f(origin.x(), origin.y(), origin.z());
    }

    std::vector<CloudPtr> checkOcclusionsDeprecated(std::vector<CloudPtr>& differenceMetaRoomToRoomClusters,
                            std::vector<CloudPtr>& differenceRoomToMetaRoomClusters, int numberOfBins = 360)
    {
        PCL_INFO("Checking occlusions");
        // Transformation to origin -> needed for spherical projection
        Eigen::Matrix4f transformFromOrigin = Eigen::Affine3f(Eigen::Translation3f(viewpoint)).matrix();
        Eigen::Matrix4f transformToOrigin = transformFromOrigin.inverse();

//        ROS_INFO_STREAM("Transform from origin "<< transformFromOrigin);
//        ROS_INFO_STREAM("Transform to origin "<< transformToOrigin);

        const double pi = std::acos(-1.0);
        std::vector<CloudPtr> toBeAdded;
        // for each cluster in the room to metaroom vector, check if it occludes anything
        // if yes, remove the occluded cluster so that it is not deleted from the metaroom
        bool clusterAdded = false;
        /*********** CLUSTERS TO BE REMOVED ************************************/
        for (size_t i=0; i<differenceRoomToMetaRoomClusters.size(); i++)
        {
            bool clusterAdded = false;
            // project clusters on sphere and check occlusions
            double thetaphi[numberOfBins][numberOfBins];
            for (size_t j=0; j<numberOfBins; j++)
            {
                for(size_t k=0; k<numberOfBins; k++)
                {
                    thetaphi[j][k] = 0.0;
                }
            }

            int occluded_clusters = 0;

            // transform cluster to Origin
            CloudPtr transformedRoomCluster(new Cloud);
            pcl::transformPointCloud (*differenceRoomToMetaRoomClusters[i], *transformedRoomCluster, transformToOrigin);

            for (size_t j=0; j<transformedRoomCluster->points.size(); j++)
            {
                // convert to spherical coordinates
                double r = sqrt(pow(transformedRoomCluster->points[j].x,2) + pow(transformedRoomCluster->points[j].y,2) + pow(transformedRoomCluster->points[j].z,2));
                double theta = pi+acos(transformedRoomCluster->points[j].z/r);
                double phi = pi+atan2(transformedRoomCluster->points[j].y,transformedRoomCluster->points[j].x);

//                ROS_INFO_STREAM("Theta: "<<theta<<";  Phi: "<<phi<<"; r: "<<r<<"; current point: "<<transformedRoomCluster->points[j].x<<" "<<transformedRoomCluster->points[j].y<<" "<<differenceRoomToMetaRoomClusters[i]->points[j].z);

                int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                int phibin = (int)((phi/(2*pi)) * numberOfBins);
                thetaphi[thetabin][phibin] = r;
            }

//            ROS_INFO_STREAM("Spherical projection computed");
            // check occlusion with other clusters
            typename std::vector<CloudPtr>::iterator metaRoomClusterIterator = differenceMetaRoomToRoomClusters.begin();
            while(metaRoomClusterIterator != differenceMetaRoomToRoomClusters.end())
            {
                int occluded = 0;
                int behind = 0;
                int infront = 0;
                // transform cluster to Origin
                CloudPtr transformedMetaRoomCluster(new Cloud);
                pcl::transformPointCloud (*(*metaRoomClusterIterator), *transformedMetaRoomCluster, transformToOrigin);

                CloudPtr pointsFront(new Cloud);
                CloudPtr pointsBehind(new Cloud);

                for (size_t k=0; k < transformedMetaRoomCluster->points.size(); k++)
                {
//                    if (k==j) continue;
                    // take spherical projection
                    double r = sqrt(pow(transformedMetaRoomCluster->points[k].x,2) + pow(transformedMetaRoomCluster->points[k].y,2) + pow(transformedMetaRoomCluster->points[k].z,2));
                    double theta = pi+acos(transformedMetaRoomCluster->points[k].z/r);
                    double phi = pi+atan2(transformedMetaRoomCluster->points[k].y,transformedMetaRoomCluster->points[k].x);

                    int thetabin = (int)((theta/(2*pi)) * numberOfBins);
                    int phibin = (int)((phi/(2*pi)) * numberOfBins);
                    if (thetaphi[thetabin][phibin] != 0.0)
                    {
                        occluded++;
                        if (thetaphi[thetabin][phibin] > r)
                        {
                            infront++;
                            pointsFront->points.push_back((*metaRoomClusterIterator)->points[k]);
                        } else {
                            behind++;
                            pointsBehind->points.push_back((*metaRoomClusterIterator)->points[k]);
                        }
                    }
                }

//                ROS_INFO_STREAM("Cluster "<<i<<" occluded "<<occluded<<" behind "<<behind<<" infront "<<infront<<"  total points  "<<transformedMetaRoomCluster->points.size());
                if (occluded != 0)
                {
                    if (/*(behind > infront)*/(behind > 0))
                    {
                        // remove this cluster so that it's not deleted from the metaroom
                        // only remove it if it's occluded by a large enough cluster
                        //                        if (transformedMetaRoomCluster->points.size() > 0.5 *(*metaRoomClusterIterator)->points.size() )
                        //                        {
                        PCL_INFO("Removing occluded points : %d and adding %d", behind, infront);
                        // segment out the occluded points which shouldn't be removed, and add the ones which are not occluded
                        CloudPtr remainingPoints(new Cloud());
                        pcl::SegmentDifferences<PointType> segment;
                        segment.setDistanceThreshold(0.001);
                        segment.setInputCloud(*metaRoomClusterIterator);
                        segment.setTargetCloud(pointsBehind);
                        segment.segment(*remainingPoints);
                        if (remainingPoints->points.size() != 0)
                        {
//                            ROS_INFO_STREAM("Removing a cluster, no points: "<<(*metaRoomClusterIterator)->points.size());
                            metaRoomClusterIterator = differenceMetaRoomToRoomClusters.erase(metaRoomClusterIterator);
                        } else {
                            metaRoomClusterIterator++;
                        }
                    } else {
                        if (!clusterAdded)
                        {
                            // only add this cluster if it's occluded by a large enough cluster
//                            if ((*metaRoomClusterIterator)->points.size() > 0.5 * differenceRoomToMetaRoomClusters[i]->points.size())
                            {
//                                ROS_INFO_STREAM("Found a cluster that needs to be added to the metaroom, no points "<<differenceRoomToMetaRoomClusters[i]->points.size());
                                toBeAdded.push_back(differenceRoomToMetaRoomClusters[i]);                                
                                clusterAdded = true;
                            }
                        }
                        metaRoomClusterIterator++;
                    }
                }
                else {
                    metaRoomClusterIterator++;
                }
            }
        }

        return toBeAdded;
    }

	/**
	 * Assuming the both points of scene and points of obstacles are already registered.
	 */
	occlusion_results checkOcclusions(CloudPtr scene, CloudPtr obstacles) {
		occlusion_results result;
		result.occluded = CloudPtr(new Cloud());
		result.nonOccluded = CloudPtr(new Cloud());

		// Transformation to origin -> needed for spherical projection
		Eigen::Affine3f transformToOrigin = Eigen::Affine3f(Eigen::Translation3f(viewpoint)).inverse();

		// init spherical map
		double thetaphi[numberOfBins][numberOfBins];
		for (size_t j = 0; j < numberOfBins; j++) {
			for (size_t k = 0; k < numberOfBins; k++) {
				thetaphi[j][k] = INFINITY;
			}
		}

		// fill spherical map by obstacles
		CloudPtr obstacles_transformed(new Cloud);
		pcl::transformPointCloud(*obstacles, *obstacles_transformed, transformToOrigin);
		for (size_t j = 0; j < obstacles_transformed->size(); j++) {
			if(!isPointValid(obstacles_transformed->at(j))) {
				continue;
			}
			// convert to spherical coordinates
			int thetabin, phibin;
			double r;
			rPhiThetaBins(obstacles_transformed->at(j), r, phibin, thetabin);
			thetaphi[thetabin][phibin] = std::min(r, thetaphi[thetabin][phibin]);
		}

		// transform cluster to Origin
		CloudPtr scene_transformed(new Cloud);
		pcl::transformPointCloud(*scene, *scene_transformed, transformToOrigin);
		for (size_t k = 0; k < scene->size(); k++) {
			if(!isPointValid(scene_transformed->at(k))) {
				continue;
			}
			// take spherical projection
			int thetabin, phibin;
			double r;
			rPhiThetaBins(scene_transformed->at(k), r, phibin, thetabin);

			if (thetaphi[thetabin][phibin] < (r + tolerance)) {
				result.occluded->push_back(scene->at(k));
			} else {
				result.nonOccluded->push_back(scene->at(k));
			}
		}

		return result;
	}

    void rPhiThetaBins(const PointType &pt, double &r, int &phi_bin, int &theta_bin) {
        r = sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
        double theta = M_PI + acos(pt.z/r);
        double phi = M_PI + atan2(pt.y,pt.x);
        theta_bin = (int)((theta/(2*M_PI)) * numberOfBins);
        phi_bin = (int)((phi/(2*M_PI)) * numberOfBins);

        theta_bin = std::min(std::max(0, theta_bin), numberOfBins-1);
        phi_bin = std::min(std::max(0, phi_bin), numberOfBins-1);
    }

	void setNumberOfBins(int numberOfBins) {
		this->numberOfBins = numberOfBins;
	}

	bool isPointValid(const PointType &pt) {
		return pcl::isFinite(pt) && !isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z);
	}

private:
    Eigen::Vector3f         viewpoint;
    int 					numberOfBins;
    float					tolerance;
};

}

#endif
