#include "utilities/utils.h"

pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipNormals(
        pcl::PointCloud<pcl::PointXYZI> & cloud,
        pcl::PointCloud<pcl::Normal> & normals){

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr zipped (new pcl::PointCloud<pcl::PointXYZINormal>());
    zipped->resize(cloud.size());

    for(uint i = 0; i < cloud.size(); i ++){
        pcl::PointXYZI & p = cloud[i];
        pcl::Normal & n = normals[i];

        pcl::PointXYZINormal & pn = (*zipped)[i];
        pn.getNormalVector4fMap() = n.getNormalVector4fMap();
        pn.getVector4fMap() = p.getVector4fMap();
        pn.intensity = p.intensity;
    }

    return zipped;
}
