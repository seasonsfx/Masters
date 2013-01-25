#ifndef CLOUDMODEL_CDATAMODEL_H_
#define CLOUDMODEL_CDATAMODEL_H_

#include <map>
#include <vector>
#include <clayer.h>
#include <cselection.h>
#include <cpointcloud.h>

class CDataModel {
 public:
    CDataModel();

    // save everything as a flat cloud?

 public:
    int last_cloud_id;
    CLayer layers;
    CSelection selection;
    std::map<int, CPointCloud> clouds;
};

#endif  // CLOUDMODEL_CDATAMODEL_H_
