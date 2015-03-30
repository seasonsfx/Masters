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
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "boost/make_shared.hpp"
#include <tuple>

#include <Eigen/Core>
#include <pcl/segmentation/extract_clusters.h>

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

    std::cout << "Size: " << idxs.size() << "Size n^2: " << idxs.size()*idxs.size() << std::endl;

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
    ec.setMinClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (smaller_cloud);
    ec.extract (cluster_indices);

    std::vector<std::vector<int> > clusters(cluster_indices.size());

    int i;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            clusters[i].push_back(idxs[*pit]);
        }

//        std::cout << "Cluster: " <<  clusters[i].size() << " data points." << std::endl;

        i++;
    }

    return clusters;

}

std::vector<int> Evaluate::concaveHull(std::vector<int> & idxs){
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

        //std::cout << "p.x: " << p.x << "p.y: " << p.y << "idxToGrid[idx]: " << idxToGrid[idx] << " height: " << height << std::endl;

        if(p.y < min_y){
            min_y = p.y;
            min_y_idx = flatcloud->size();
        }

        flatcloud->push_back(p);
    }

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(flatcloud);

    int K = 10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    int current_idx = min_y_idx;

    Eigen::Vector2f grad(1.f, 0.f);

    std::vector<int> hull = {idxs[min_y_idx]};
    std::set<int> visited;
    int i = 0;

    auto vecToStr = [](Eigen::Vector2f vec){
        std::string ret = std::string("(") + std::to_string(float(vec.x())) + std::string(" ") + std::to_string(float(vec.y())) + std::string(")");
        return ret;
    };

    while(1){
        pcl::PointXY & currentPoint = flatcloud->at(current_idx);
        kdtree.nearestKSearch(currentPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

        int min_angle = M_PI*2;
        int min_angle_idx = -1;

        std::cout << "=======================================================" << std::endl;

        for(int kidx: pointIdxNKNSearch){

            if(kidx == current_idx || visited.find(kidx) != visited.end()){
                continue;
            }

            pcl::PointXY & p = flatcloud->at(kidx);

            Eigen::Vector2f to(p.x, p.y);
            Eigen::Vector2f from(currentPoint.x, currentPoint.y);


            Eigen::Vector2f gradTo = (from + grad);
            float fromToDist = (to - from).norm();

            float angle = acos((gradTo - from).dot((to - from)/fromToDist));

            std::cout << "------------------------------------------------------" << std::endl;
            std::cout << "current gradient; " << vecToStr(grad) << std::endl;
            std::cout << "current_idx: " << current_idx << ", kidx: " << kidx << std::endl;
            std::cout << "from: " << vecToStr(from) << " to:  " << vecToStr(to) << " gradTo: " << vecToStr(gradTo) << std::endl;
            std::cout << "(gradTo - from): " << vecToStr(gradTo - from) << " dot with (to - from)/fromToDist: " << vecToStr((to - from)/fromToDist) << std::endl;
            std::cout << "angle: " << angle << std::endl;

            if(angle < min_angle){
                min_angle = angle;
                min_angle_idx = kidx;
                current_idx = kidx; // next round idx
                std::cout << "Min Angle! " <<  kidx << std::endl;
            }

            // New gradient
            Eigen::Vector2f diff = (to - from);
            grad = diff / diff.norm();


        }

        if(min_angle_idx == -1){
            std::cout << "bad hull!!!!!!!" << std::endl;
            return hull;
        }

        // If we are at the starting point
        if(min_angle_idx == min_y_idx){
            std::cout << "good hull!!!!!!!" << std::endl;
            break;
        }

        pcl::PointXY & next_point = flatcloud->at(min_angle_idx);
        std::cout << "Next point: (" << next_point.x << ", " << next_point.y << ")" << std::endl;
        std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

        hull.push_back(idxs[min_angle_idx]);
        visited.insert(min_angle_idx);

        // Remove the first point
        if(i == 4){
           visited.erase(min_y_idx);
        }
        i++;
        //std::cout << "Hull size: " << hull.size() << endl;
    }

    return hull;
}

void Evaluate::paint2d(){
    // 0 , 0 is not used
    lasso_->drawLasso(0, 0, flatview_);
}

void Evaluate::lassoPoints(std::vector<int> & idxs){

    if(cl_->clouds_.size() == 0){
        std::cout << "No clouds" << std::endl;
        return;
    }

    const std::vector<int> & idxToGrid = cl_->active_->cloudToGridMap();
    int height = cl_->active_->scan_height();

    //lasso_->clear();

    //Eigen::Affine2f t = flatview_->getNDCCamera();

    std::vector<Eigen::Vector2f> polygon;

    // Create a new cloud with only the 2d idx points
    for(int idx : idxs){
        polygon.push_back(Eigen::Vector2f (idxToGrid[idx] / height, idxToGrid[idx] % height));
//        Eigen::Vector2f ndc_point = t * Lasso::NDCPoint(
//                    p,
//                    cl_->active_->scan_width(),
//                    cl_->active_->scan_height());

//       lasso_->addNormPoint(ndc_point);

    }

    polygon.push_back(polygon[0]);

    boost::shared_ptr<std::vector<int>> selected_indices = boost::make_shared<std::vector<int>>();

    for (uint idx = 0; idx < cl_->active_->size(); idx++) {
        Eigen::Vector2f point(Eigen::Vector2f (idxToGrid[idx] / height, idxToGrid[idx] % height));

        if(pointInsidePolygon(polygon, point)){
            selected_indices->push_back(idx);
        }
    }

    QApplication::processEvents();


//    lasso_->getIndices2D(cl_->active_->scan_height(), flatview_->getCamera(),
//                         cl_->active_->cloudToGridMap(), selected_indices);

    std::cout << "Points selected: " << selected_indices->size() << std::endl;

    for(int sidx : *selected_indices){
        std::cout << "index selected: " << sidx << ", ";
    }

    std::cout << std::endl;

    //core_->us_->beginMacro("Lasso");
    core_->us_->push(new Select(cl_->active_, selected_indices, false, core_->mw_->select_mask_, true, ll_->getHiddenLabels()));
    //core_->us_->endMacro();
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

    // collect truth idxs
    // find false positives
    // find false negatives
    // cluster false positives
    // cluster false negatives
    // lasso


    int count = 0;
    int count1 = 0;
    int count2 = 0;

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

    core_->us_->beginMacro("Eval tool selection");
         // Clear the selection mask
        core_->us_->push(new Select(cl_->active_, all_idxs, true, 0xff, true));

        // Reproduce the saved selection
        core_->us_->push(new Select(cl_->active_, select_idxs, false, 1, true));


        // get false_positive, false_negative
        std::tie(false_positive, false_negative) = get_false_selections(world_points, target_mask);

        // Clear selection
        core_->us_->push(new Select(cl_->active_, all_idxs, true, 0xff, true));


        std::vector<std::vector<int> > fps = cluster(false_positive);
        for(std::vector<int> & fp : fps){
            //core_->us_->push(new Select(cl_->active_, boost::make_shared<std::vector<int>>(fp), false, 2, true));

            std::vector<int> hull = concaveHull(fp);
            std::cout << "Hull size: " << hull.size() << " around " << fp.size() << " points." << std::endl;
            //lassoPoints(hull);
            core_->us_->push(new Select(cl_->active_, boost::make_shared<std::vector<int>>(hull), false, 4, true));
        }

        std::vector<std::vector<int> > fns = cluster(false_negative);
        for(std::vector<int> & fn : fns){
            std::vector<int> hull = concaveHull(fn);
            //lassoPoints(hull);
            core_->us_->push(new Select(cl_->active_, boost::make_shared<std::vector<int>>(hull), false, 4, true));
        }

    core_->us_->endMacro();


    // http://en.wikipedia.org/wiki/Precision_and_recall
    qDebug() << "overlap: " << count << "truth: " << count1 << "result: " << count2;
    qDebug() << "Recall: " << float(count)/count1;
    qDebug() << "Precision: " << float(count)/count2;

    recall_->setText(QString("%1").arg(float(count)/count1));
    precision_->setText(QString("%1").arg(float(count)/count2));
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
