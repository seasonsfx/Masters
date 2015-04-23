#include "plugins/evaluate/evaluate.h"
#include <QApplication>
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QLineEdit>
#include <QMessageBox>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "boost/make_shared.hpp"
#include <tuple>
#include <thread>
#include <chrono>
#include <stack>

#include <Eigen/Core>
#include <pcl/segmentation/extract_clusters.h>

float SIMPLIFY = 1.5;
float EXPAND = SIMPLIFY + 0.5;

QString Evaluate::getName(){
    return "Evaluate";
}

void Evaluate::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    is_enabled_ = false;
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    enable_ = new QAction(QIcon(":/evaluate.png"), "Enable Evaluate", 0);
    enable_->setCheckable(true);
    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));

    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    layout->addWidget(new QLabel("World:", settings_));
    QListWidget * l0 = new QListWidget(settings_);
    layout->addWidget(l0);
    QHBoxLayout * split0 = new QHBoxLayout(settings_);
    layout->addLayout(split0);
    QPushButton * add0 = new QPushButton("Add selected layers", settings_);
    split0->addWidget(add0);
    QPushButton * clear0 = new QPushButton("Clear", settings_);
    split0->addWidget(clear0);

    layout->addWidget(new QLabel("Truth:", settings_));
    QListWidget * l1 = new QListWidget(settings_);
    layout->addWidget(l1);
    QHBoxLayout * split1 = new QHBoxLayout(settings_);
    layout->addLayout(split1);
    QPushButton * add1 = new QPushButton("Add selected layers", settings_);
    split1->addWidget(add1);
    QPushButton * clear1 = new QPushButton("Clear", settings_);
    split1->addWidget(clear1);

    layout->addWidget(new QLabel("Segementation:", settings_));
    QListWidget * l2 = new QListWidget(settings_);
    layout->addWidget(l2);
    QHBoxLayout * split2 = new QHBoxLayout(settings_);
    layout->addLayout(split2);
    QPushButton * add2 = new QPushButton("Add selected layers", settings_);
    split2->addWidget(add2);
    QPushButton * clear2 = new QPushButton("Clear", settings_);
    split2->addWidget(clear2);

    QPushButton * evaluate = new QPushButton("Evaluate", settings_);
    layout->addWidget(evaluate);

    precision_ = new QLineEdit(settings_);
    precision_->setReadOnly(true);
    recall_ = new QLineEdit(settings_);
    recall_->setReadOnly(true);

    QHBoxLayout * split = new QHBoxLayout(settings_);
    layout->addLayout(split);
    split->addWidget(new QLabel("Recall", settings_));
    split->addWidget(recall_);
    split->addWidget(new QLabel("Precision", settings_));
    split->addWidget(precision_);

    // connect

    auto alreadySelected = [=] (boost::weak_ptr<Layer> s) {
        bool found = false;

        for(auto layers : std::vector<decltype(world_layers_)> {world_layers_, target_layers_, selection_layers_}){
            for(boost::weak_ptr<Layer> e : layers){
                if(s.lock() == e.lock()){
                    found = true;
                    break;
                }
            }
        }

        return found;
    };

    connect(add0, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){

            bool found = alreadySelected(s);

            if(!found){
                world_layers_.push_back(s);
                l0->addItem(s.lock()->getName());
            }
        }
    });


    connect(add1, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
           bool found = alreadySelected(s);
            if(!found){
                target_layers_.push_back(s);
                l1->addItem(s.lock()->getName());
            }
        }
    });

    connect(add2, &QPushButton::clicked, [=] (){
        for(boost::weak_ptr<Layer> s : ll_->getSelection() ){
            bool found = alreadySelected(s);
            if(!found){
                selection_layers_.push_back(s);
                l2->addItem(s.lock()->getName());
            }
        }
    });

    connect(clear0, &QPushButton::clicked, [=] (){
        l0->clear();
        world_layers_.clear();
    });

    connect(clear1, &QPushButton::clicked, [=] (){
        l1->clear();
        target_layers_.clear();
    });

    connect(clear2, &QPushButton::clicked, [=] (){
        l2->clear();
        selection_layers_.clear();
    });

    connect(evaluate, SIGNAL(clicked()), this, SLOT(eval()));

    layout->addStretch();

    lasso_ = new Lasso();
}

std::tuple<std::vector<int>, std::vector<int> > Evaluate::get_false_selections(std::vector<int> & world_idxs, std::vector<bool> & target_mask){

    std::vector<int> false_positive;
    std::vector<int> false_negative;

    for(int i : world_idxs){

        bool selected = uint8_t(cl_->active_->flags_[i]) & uint8_t(0xff);

        if(!selected && target_mask[i]){
            false_negative.push_back(i);
        } else if(selected && !target_mask[i]) {
            false_positive.push_back(i);
        }
    }

    return std::make_tuple(false_positive, false_negative);
}

std::vector<std::vector<int> > Evaluate::cluster(std::vector<int> & idxs){
    // for every index find the closest point

    //std::cout << "Size: " << idxs.size() << "Size n^2: " << idxs.size()*idxs.size() << std::endl;

    // make a smaller cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    for(int i = 0; i < idxs.size(); i++){
       smaller_cloud->push_back(cl_->active_->at(idxs[i]));
    }


    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (smaller_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.02); // 2cm
    //ec.setMinClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (smaller_cloud);
    ec.extract (cluster_indices);

    std::vector<std::vector<int> > clusters(cluster_indices.size());

    int i;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            clusters[i].push_back(idxs[*pit]);
        }

        i++;
    }

    return clusters;

}

Eigen::Vector2f Evaluate::getPoint(int idx){
    const std::vector<int> & idxToGrid = cl_->active_->cloudToGridMap();
    int height = cl_->active_->scan_height();

    float x = idxToGrid[idx] / height;
    float y = idxToGrid[idx] % height;
    Eigen::Vector2f p(x, y);
    return p;
}


int Evaluate::dpR(std::vector<int> & idxs, std::vector<bool> & keep, int start_idx, int end_idx, float e){

    keep[start_idx] = true;
    keep[end_idx] = true;

    if(end_idx-start_idx < 2){
        return -1;
    }

    Eigen::Vector2f start = getPoint(idxs[start_idx]);
    Eigen::Vector2f end = getPoint(idxs[end_idx]);


    // Find the point with the maximum distance
    // Exclude the start end end points
    int max_dist_idx = -1;
    float max_dist = 0;

    for(int i = start_idx+1; i < end_idx; i++){
        Eigen::Vector2f p = getPoint(idxs[i]);

        Eigen::Vector2f a = p-start;
        Eigen::Vector2f b = end-start;

        Eigen::Vector2f proj = (b/b.norm()) * (a.dot(b) / b.norm());

        float dist = ((start + proj) - p).norm();

        if(dist > max_dist){
            max_dist_idx = i;
            max_dist = dist;
        }
    }

    bool keep_it = max_dist > e;

    if(keep_it){
        keep[max_dist_idx] = true;
    } else {
        return -1;
    }


    return max_dist_idx;
}

std::vector<int> Evaluate::dp(std::vector<int> & idxs, float e){
    int start = 0;
    int end = idxs.size()-1;

    std::vector<bool> keep(idxs.size(), false);
    std::vector<int> simplified;

    std::stack<std::tuple<int, int>> stack;
    stack.push(std::make_tuple(start, end));

    while(stack.size() > 0){
        std::tuple<int, int> bounds = stack.top();
        stack.pop();

        int left = std::get<0>(bounds);
        int right = std::get<1>(bounds);

        int max = dpR(idxs, keep, left, right, e);

        if(max != -1){
            stack.push(std::make_tuple(left, max));
            stack.push(std::make_tuple(max, right));
        }
    }


    // Return kept values
    for(int i = 0; i < idxs.size(); i++){
        if(keep[i]){
            simplified.push_back(idxs[i]);
        }
    }

    std::cout << "input: " << idxs.size() << ", output: " << simplified.size() << std::endl;

    return simplified;
}

auto pointLocation = [](Eigen::Vector2f A, Eigen::Vector2f B, Eigen::Vector2f P) {
    int cp1 = (B.x()-A.x())*(P.y()-A.y()) - (B.y()-A.y())*(P.x()-A.x());
    return (cp1>0)?1:-1;
};

auto distance = [](Eigen::Vector2f A, Eigen::Vector2f B, Eigen::Vector2f C) {
    int ABx = B.x()-A.x();
    int ABy = B.y()-A.y();
    int num = ABx*(A.y()-C.y())-ABy*(A.x()-C.x());
    if (num < 0) num = -num;
    return num;
};


std::vector<int> Evaluate::cvR(Eigen::Vector2f a, Eigen::Vector2f b, std::vector<int> & idxs){
    // Find the point futhest away from the line in each group

    if(idxs.size() == 0){
        return std::vector<int>();
    }

    if(idxs.size() == 1){
        return idxs;
    }


    auto maxDistIdx = [&](std::vector<int> & idxs, Eigen::Vector2f & start, Eigen::Vector2f & end){
        int max_dist_idx = -1;
        float max_dist = 0;

        for(int idx : idxs){
            Eigen::Vector2f p = getPoint(idx);
            float dist = distance(start, end, p);

            if(dist > max_dist){
                max_dist_idx = idx;
                max_dist = dist;
            }
        }
        return max_dist_idx;
    };


    int max_idx = maxDistIdx(idxs, a, b);
    Eigen::Vector2f m = getPoint(max_idx);

    std::vector<int> left;
    std::vector<int> right;

    for(int idx : idxs){
        if(idx == max_idx){
            continue;
        }

        Eigen::Vector2f p = getPoint(idx);

        if(pointLocation(a,p,m) == 1){
            left.push_back(idx);
        } else if(pointLocation(p,b,m) == 1) {
            right.push_back(idx);
        }
    }

    idxs.clear();// Save memory

    std::vector<int> left_out;
    std::vector<int> right_out;

    if (left.size() > 0) {
        left_out = cvR(a, m, left);
    }

    left_out.push_back(max_idx);

    if (right.size() > 0) {
        right_out = cvR(m, b, right);
    }

    left_out.insert(left_out.end(), right_out.begin(), right_out.end());

    return left_out;

}

std::vector<int> Evaluate::convexHull(std::vector<int> & idxs){

    // Find min max
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();

    int min_x_idx = -1;
    int max_x_idx = -1;

    for(int idx : idxs){
        Eigen::Vector2f p = getPoint(idx);

        if(p.x() < min_x){
            min_x = p.x();
            min_x_idx = idx;
        }

        if(p.x() > max_x){
            max_x = p.x();
            max_x_idx = idx;
        }
    }

    // Find points on oposite sides of the line
    Eigen::Vector2f a = getPoint(min_x_idx);
    Eigen::Vector2f b = getPoint(max_x_idx);

    std::vector<int> left;
    std::vector<int> right;

    for(int idx : idxs){

        // Skip the line ends
        if(idx == min_x_idx || idx == max_x_idx){
            continue;
        }

        Eigen::Vector2f p = getPoint(idx);

        if (pointLocation(a,b,p) == -1){
            left.push_back(idx);
        }
        else {
            right.push_back(idx);
        }

    }

    // std::cout << "Left/right/all: " << left.size() << "/" << right.size() << "/" << idxs.size() << std::endl;

    std::vector<int> left_out = cvR(a, b, right);
    std::vector<int> right_out = cvR(b, a, left);

    left_out.insert(left_out.begin(), max_x_idx);
    left_out.push_back(min_x_idx);
    left_out.insert(left_out.end(), right_out.begin(), right_out.end());


    std::cout << "Hull" << std::endl;
    for(int idx : left_out){
        Eigen::Vector2f p = getPoint(idx);
        std::cout << p.x() << ", " << p.y() << std::endl;
    }
    std::cout << "------------------" << std::endl;

    return left_out;
}

    // Sort hull

    // center off mass
//    Eigen::Vector2f center(0, 0);
//    for(int idx : hull){
//        center += getPoint(idx);
//    }
//    center /= hull.size();

//    // sort
//    std::sort(hull.begin(), hull.end(), [this, &center](int a, int b) -> bool {
//        Eigen::Vector2f p1 = getPoint(a) - center;
//        Eigen::Vector2f p2 = getPoint(b) - center;
//        return atan2(p1.y(), p1.x()) > atan2(p2.y(), p2.x());
//    });

//    std::cout << "Hull" << std::endl;
//    for(int idx : hull){
//        Eigen::Vector2f p = getPoint(idx);
//        std::cout << p.x() << ", " << p.y() << std::endl;
//    }
//    std::cout << "------------------" << std::endl;

//    return hull;


//    auto vecToStr = [](Eigen::Vector2f vec){
//        std::string ret = std::string("(") + std::to_string(float(vec.x())) + std::string(" ") + std::to_string(float(vec.y())) + std::string(")");
//        return ret;
//    };

std::vector<int> Evaluate::concaveHull(std::vector<int> & idxs, float simplify){
    const std::vector<int> & idxToGrid = cl_->active_->cloudToGridMap();
    int height = cl_->active_->scan_height();

    pcl::PointCloud<pcl::PointXY>::Ptr flatcloud = boost::make_shared<pcl::PointCloud<pcl::PointXY> >();

    float min_y = height;
    int min_y_idx = -1;

    // Create a new cloud with only the 2d idx points
    for(int idx : idxs){
        pcl::PointXY p;
        p.x = idxToGrid[idx] / height;
        p.y = idxToGrid[idx] % height;

        if(p.y < min_y){
            min_y = p.y;
            min_y_idx = flatcloud->size();
        }

        flatcloud->push_back(p);
    }

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(flatcloud);

    int K = 9;//49;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    int current_idx = min_y_idx;

    Eigen::Vector2f grad(1.f, 0.f);

    std::vector<int> hull = {idxs[min_y_idx]};
    std::set<int> visited;
    int i = 0;

    // a is initial
    auto isRightHandTurn = [](Eigen::Vector2f a, Eigen::Vector2f b){
       return (b.y() * a.x() - b.x() * a.y()) > 0;
    };

    while(true){
        pcl::PointXY & currentPoint = flatcloud->at(current_idx);
        Eigen::Vector2f from(currentPoint.x, currentPoint.y);
//        std::cout << "Current point: (" << currentPoint.x << ", " << currentPoint.y << ")" << std::endl;
        pointIdxNKNSearch.clear();
        pointNKNSquaredDistance.clear();
        kdtree.nearestKSearch(currentPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        float max_angle = 0;
        int max_angle_idx = -1;

        for(int kidx: pointIdxNKNSearch){

            if(kidx == current_idx || visited.find(kidx) != visited.end()){
                continue;
            }

            pcl::PointXY & p = flatcloud->at(kidx);

            Eigen::Vector2f to(p.x, p.y);

            Eigen::Vector2f heading = to - from;

            float headingDist = heading.norm();

            float angle = acos((-grad).dot(heading/headingDist));

            if(isRightHandTurn(grad, heading/headingDist)){
                angle = M_PI*2 - angle;
            }

            if(angle > max_angle){
                max_angle = angle;
                max_angle_idx = kidx;
            }

        }


        if(max_angle_idx == -1){
            std::cout << "bad hull!!!!!!!" << std::endl;
            return hull;
        }

        // New gradient
        auto to_ = flatcloud->at(max_angle_idx);
        Eigen::Vector2f to(to_.x, to_.y);
        Eigen::Vector2f diff = (to - from);
        grad = diff / diff.norm();
        current_idx = max_angle_idx;

        // If we are at the starting point
        if(max_angle_idx == min_y_idx){
            std::cout << "good hull!!!!!!!" << std::endl;
            break;
        }

        hull.push_back(idxs[max_angle_idx]);
        visited.insert(max_angle_idx);

        // Remove the first point
        if(i == 4){
           visited.erase(min_y_idx);
        }
        i++;
    }

    return dp(hull, simplify);
}

void Evaluate::paint2d(){
}

std::vector<Eigen::Vector2f> Evaluate::makePolygon(std::vector<int> & idxs, float expand){
    std::vector<Eigen::Vector2f> polygon;

    int size = idxs.size();
    for(int i = 0; i < size; i++){
        Eigen::Vector2f before = getPoint(i > 0 ? idxs[i-1] : idxs[size+i-1]);
        Eigen::Vector2f current = getPoint(idxs[i]);
        Eigen::Vector2f after = getPoint(idxs[(i+1)%size]);

        Eigen::Vector2f point_gradient = ((current - before) + (after - current));
        point_gradient = point_gradient/point_gradient.norm();

        Eigen::Vector2f point_normal(-point_gradient.y(), point_gradient.x());

        polygon.push_back(current + (point_normal *  expand));
    }
    polygon.push_back(polygon[0]);
    return polygon;
}

boost::shared_ptr<std::vector<int> > Evaluate::lassoPoints(std::vector<int> & idxs, float expand){

    boost::shared_ptr<std::vector<int> > selected_indices = boost::make_shared<std::vector<int>>();

    if(cl_->clouds_.size() == 0) {
        std::cout << "No clouds" << std::endl;
        return  selected_indices;
    }

    std::vector<Eigen::Vector2f> polygon = makePolygon(idxs, expand);

    for (uint idx = 0; idx < cl_->active_->size(); idx++) {
        Eigen::Vector2f point(getPoint(idx));

        if(pointInsidePolygon(polygon, point)){
            selected_indices->push_back(idx);
        }
    }

    return selected_indices;
}


void Evaluate::eval() {
    auto is_label_in_set = [=] (uint16_t label, std::vector<boost::weak_ptr<Layer> > & layers){
        const LayerSet & ls = ll_->getLayersForLabel(label);

        for(Layer * x : ls){
            for(boost::weak_ptr<Layer> y: layers){
                if(y.lock().get() == x)
                    return true;
            }
        }

        return false;
    };

    // collect the world idxs



    std::vector<int> world_points(cl_->active_->points.size());
    std::vector<bool> target_mask(cl_->active_->points.size());

    auto all_idxs = boost::make_shared<std::vector<int>>();
    auto select_idxs = boost::make_shared<std::vector<int>>();

    for(uint i = 0; i < cl_->active_->labels_.size(); i++){
        uint16_t label = cl_->active_->labels_[i];

        bool in_world = is_label_in_set(label, world_layers_);
        bool in_target = is_label_in_set(label, target_layers_);
        bool in_select = is_label_in_set(label, selection_layers_);

        if(in_world){
            world_points.push_back(i);
        }

        target_mask[i] = in_target;

        all_idxs->push_back(i);
        if(in_select){
            select_idxs->push_back(i);
        }
    }

    std::vector<int> false_positive;
    std::vector<int> false_negative;


    int MAX_IGNORE = 10;
    int lasso_count = 0;
    int lasso_vertex_count = 0;

    auto lassoHull = [&] (std::vector<int> & hull, bool deselect) {

//        QMessageBox msgBox;
//        msgBox.setText(QString("lasso cluster of size ") + QString::number(cluster.size()));
//        msgBox.exec();

        lasso_count++;
        lasso_vertex_count += hull.size();


        boost::shared_ptr<std::vector<int>> selected_indices = lassoPoints(hull, EXPAND);

        if(selected_indices->size() == 0){
            std::cout << "ZERO!!!!! Points selected!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " <<  std::endl;
            return false;
        }

        core_->us_->push(new Select(cl_->active_, selected_indices, deselect, 1, true, ll_->getHiddenLabels()));

        std::cout << "Points selected: " << selected_indices->size() << std::endl;

        return true;
    };

    core_->us_->beginMacro("Eval tool selection");

    // Reproduce the saved selection
    core_->us_->push(new Select(cl_->active_, all_idxs, true, 0xff, true));
    core_->us_->push(new Select(cl_->active_, select_idxs, false, 1, true));

    QApplication::processEvents();
    std::this_thread::sleep_for(std::chrono::seconds(2));


    bool changes_made = true;
    while(changes_made){
        // While there are false clusters with more than MAX_IGNORE points

        std::cout << "Iteration --------------------------------------" << std::endl;

        changes_made = false;
        // get false_positive, false_negative
        std::tie(false_positive, false_negative) = get_false_selections(world_points, target_mask);


        // Create clusters
        std::vector<std::vector<int> > false_positive_clusters = cluster(false_positive);
        std::vector<std::vector<int> > false_negative_clusters = cluster(false_negative);


        std::vector<std::vector<int> > false_positive_hulls;
        std::vector<std::vector<int> > false_negative_hulls;

        // Create concave hulls
        for(std::vector<int> & cluster : false_positive_clusters){
            if(cluster.size() <= MAX_IGNORE){
                continue;
            }

            std::vector<int> hull = concaveHull(cluster, SIMPLIFY);

            if(hull.size() == cluster.size()){
                std::cout << "hull size === cluster size" <<  std::endl;
                continue;
            }

            false_positive_hulls.push_back(hull);
        }

        for(std::vector<int> & cluster : false_negative_clusters){
            if(cluster.size() <= MAX_IGNORE){
                continue;
            }

            std::vector<int> hull = concaveHull(cluster, SIMPLIFY);

            if(hull.size() == cluster.size()){
                std::cout << "hull size === cluster size" <<  std::endl;
                continue;
            }

            false_negative_hulls.push_back(hull);
        }


        auto hasFalseSelection = [this, &target_mask](boost::shared_ptr<std::vector<int> > idxs, bool deselect, std::vector<bool> & selection_mask) {
            int threshold = 10;
            int false_selections = 0;
            for(int idx: *idxs){
                bool is_selected = selection_mask[idx];
                bool should_be_selected = target_mask[idx];
                if((deselect && should_be_selected && is_selected) || (!deselect && !should_be_selected && !is_selected)){
                    false_selections++;
                    if(false_selections > threshold){
                        return true;
                    }
                }
            }

            return false;
        };

        // Copy current selection
        std::vector<bool> copied_selection(cl_->active_->flags_.size());
        for(int i = 0; i < cl_->active_->flags_.size(); i++){
            copied_selection[i] = uint8_t(cl_->active_->flags_[i]) & uint8_t(0xff);
        }

        // Try to combine hulls
        auto combineConcaveHulls = [this, &hasFalseSelection, &copied_selection](std::vector<std::vector<int> > & hulls, bool deselect){
            // We can cobine two concave hulls if the convex hull around them does not create more false selections

            // For each of the hulls apply the selection to a copy of the original selection
            for(int i = 0; i < hulls.size(); i++) {
                boost::shared_ptr<std::vector<int> > selected_idxs = lassoPoints(hulls[i], EXPAND);
                // Select
                for(int idx : *selected_idxs){
                    copied_selection[idx] = !deselect;
                }

            }

            // Combine hulls if false selections do not occur
            for(int i = 0; i < hulls.size(); i++) {
                for(int j = i+1; j < hulls.size(); j++) {
                    std::vector<int> combined;
                    combined.insert(combined.end(), hulls[i].begin(), hulls[i].end());
                    combined.insert(combined.end(), hulls[j].begin(), hulls[j].end());
                    //std::cout << "Combined size: " << combined.size() << " I size: " <<  hulls[i].size() << " J size: " << hulls[j].size() << std::endl;
                    std::vector<int> chull = convexHull(combined);
                    boost::shared_ptr<std::vector<int> > selected_idxs = lassoPoints(chull, 0);
                    if(!hasFalseSelection(selected_idxs, deselect, copied_selection)){
                        hulls[i] = chull;
                        hulls.erase(hulls.begin()+j);
                        j--;
                    }
                }
            }
        };

        //std::cout << "false_positive_hulls before: " << false_positive_hulls.size() << std::endl;
        combineConcaveHulls(false_positive_hulls, true);
        //std::cout << "false_positive_hulls after: " << false_positive_hulls.size() << std::endl;
        //std::cout << "false_negative_hulls before: " << false_positive_hulls.size() << std::endl;
        combineConcaveHulls(false_negative_hulls, false);
        //std::cout << "false_negative_hulls after: " << false_positive_hulls.size() << std::endl;

        // Lasso the hulls
        for(std::vector<int> & hull : false_positive_hulls){
            //std::cout << "Hull size: " << hull.size() << " around " << cluster.size() << " points." << std::endl;

            core_->us_->push(new Select(cl_->active_, boost::make_shared<std::vector<int> >(hull), false, 2, true));

            changes_made = lassoHull(hull, true) || changes_made;
            QApplication::processEvents();
            //std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        for(std::vector<int> & hull : false_negative_hulls){
            core_->us_->push(new Select(cl_->active_, boost::make_shared<std::vector<int> >(hull), false, 4, true));

            changes_made = lassoHull(hull, false) || changes_made;
            QApplication::processEvents();
            //std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        std::cout << "Changes made: " << changes_made << std::endl;
        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    }

    core_->us_->endMacro();

    std::cout << "Lasso actions: " << lasso_count << std::endl;
    std::cout << "Total lasso vertices: " << lasso_vertex_count << std::endl;

    float seconds_per_vertex = 1;
    float seconds_per_lasso = 4;


    std::cout << "Estimated user time (seconds): " << lasso_count*seconds_per_lasso + lasso_vertex_count*seconds_per_vertex << std::endl;


//    recall_->setText(QString("%1").arg(float(count)/count1));
//    precision_->setText(QString("%1").arg(float(count)/count2));
}

void Evaluate::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->tooloptions_->removeWidget(settings_);
    delete enable_;
}

Evaluate::~Evaluate(){
    
}

void Evaluate::enable() {
    if(is_enabled_){
        disable();
        return;
    }

    lasso_->clear();

    connect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint2d()),
            Qt::DirectConnection);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);
    emit enabling();
    glwidget_->installEventFilter(this);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Evaluate::disable(){

    lasso_->clear();
    disconnect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint2d()));

    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(this);
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.evaluate")
