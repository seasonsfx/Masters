#include "plugins/markov/markov.h"
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QMessageBox>
#include <functional>
#include <boost/serialization/shared_ptr.hpp>
#include <pcl/search/kdtree.h>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "plugins/markov/mincut.h"
#include "plugins/normalestimation/normalestimation.h"
#include "utilities/utils.h"
#include "utilities/picker.h"
#include "utilities/cv.h"

#include "data.h"
#include "onlinetree.h"
#include "onlinerf.h"

QString Markov::getName(){
    return "markov";
}

void Markov::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

}

void Markov::initialize2(PluginManager * pm) {
    ne_ = pm->findPlugin<NormalEstimator>();
    if (ne_ == nullptr) {
        qDebug() << "Normal estimator plugin needed for markov";
        return;
    }

    enable_ = new QAction(QIcon(":/markov.png"), "markov action", 0);
    enable_->setCheckable(true);

    connect(enable_,&QAction::triggered, [this] (bool on) {
        graphcut();
    });

    mw_->toolbar_->addAction(enable_);
    std::function<void(int)> func = std::bind(&Markov::graphcut, this, std::placeholders::_1);
    picker_ = new Picker(glwidget_, cl_, func);

    forrest_action_ = new QAction(QIcon(":/randomforest.png"), "forrest action", 0);
    connect(forrest_action_, &QAction::triggered, [this] (bool on) {
        randomforest();
    });

    mw_->toolbar_->addAction(forrest_action_);


    enabled_ = false;
    fg_idx_ = -1;
}

void Markov::cleanup(){
    mw_->toolbar_->removeAction(enable_);
    mw_->toolbar_->removeAction(forrest_action_);
    delete enable_;
    delete picker_;
    delete forrest_action_;
}

Markov::~Markov(){
    qDebug() << "Markov deleted";
}

void Markov::enable() {
    if(enabled_){
        disable();
        return;
    }

    QTabWidget * tabs = qobject_cast<QTabWidget *>(glwidget_->parent()->parent());
    tabs->setCurrentWidget(glwidget_);
    enable_->setChecked(true);

    emit enabling();

    glwidget_->installEventFilter(picker_);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));

    enabled_ = true;
    // Let the user know what to do
    QMessageBox::information(nullptr, tr("Select foreground"),
                    tr("Select the center of an object..."),
                    QMessageBox::Ok, QMessageBox::Ok);


}

void Markov::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    glwidget_->removeEventFilter(picker_);
    enabled_ = false;
}

void Markov::graphcut(int idx){
    qDebug() << "Myfunc";
/*
    if(fg_idx_ == -1) {
        fg_idx_ = idx;
        QMessageBox::information(nullptr, tr("Select background"),
                        tr("Select background"),
                        QMessageBox::Ok, QMessageBox::Ok);
        return;
    }
*/
    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;


    // Downsample
    std::vector<int> pca_idxs;

    pcl::PointCloud<pcl::PointXYZI>::Ptr smaller_cloud = octreeDownsample(cloud.get(), 0.02, pca_idxs);

    // PCA
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smaller_cloud.get(), 0.2f, 0);

    // Determine veg
    std::vector<bool> likely_veg(smaller_cloud->size());

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f eig = (*pca)[i];

        // If not enough neighbours
        if(eig[1] < eig[2]) {
            likely_veg[i] = false;
            continue;
        }

        float eig_sum = eig.sum();

        eig /= eig_sum;

        float fudge_factor = 5.0f;
        if(eig[1] < 0.05 * fudge_factor || eig[2] < 0.01 * fudge_factor) {
            likely_veg[i] = false;
        } else {
            likely_veg[i] = true;
        }

    }



    // Downsample for graph cut
    std::vector<int> big_to_small_map;
    smaller_cloud = octreeDownsample(cloud.get(), 0.1, big_to_small_map);

    std::set<int> foreground_points;

    // Map veg to foreground points in 2nd downsampled cloud
    for(uint i = 0; i < cloud->size(); i++) {
        uint pca_idx = pca_idxs[i];
        if(likely_veg[pca_idx] == true){
            uint small_idx = big_to_small_map[i];
            foreground_points.insert(small_idx);
        }
    }


    // Set up graph cut

    MinCut mc;
    mc.setInputCloud(smaller_cloud);


    //foreground_points.push_back(big_to_small_map[fg_idx_]);

    //std::vector<int> background_points;
    //background_points.push_back(big_to_small_map[idx]);

    double radius = 3.0;
    double sigma = 0.25;
    int neigbours = 10;
    double source_weight = 0.8;

    mc.setForegroundPoints (foreground_points);
    //mc.setBackgroundPoints(background_points);
    mc.setRadius (radius);
    mc.setSigma (sigma);
    mc.setNumberOfNeighbours (neigbours);
    mc.setSourceWeight (source_weight);

    std::vector<pcl::PointIndices> clusters;
    mc.extract (clusters);

    auto select = boost::make_shared<std::vector<int>>();

    // GAH!!!! inefficient

    std::vector<bool> small_idxs_selected(smaller_cloud->size(), false);
    for(int idx : clusters[1].indices){
        small_idxs_selected[idx] = true;
    }

    for(size_t i = 0; i < big_to_small_map.size(); i++) {
        int idx = big_to_small_map[i];
        if(small_idxs_selected[idx]) {
            select->push_back(i);
        }
    }

    //std::copy(clusters[1].indices.begin(), clusters[1].indices.end(), select->begin());

    Select * selectCmd = new Select(cl_->active_, select);



    core_->us_->beginMacro("Markov min cut");
    core_->us_->push(selectCmd);
    core_->us_->endMacro();
    core_->cl_->updated();
    core_->mw_->stopBgAction("Cut completed.");


    // We need a layer for the region of interest

    // Need a layer for the pixels pinned to the fg

    // Need a layer for the pixels pinned to the bg
    fg_idx_ = -1;
    disable();
}

double timeIt(int reset) {
    static time_t startTime, endTime;
    static int timerWorking = 0;

    if (reset) {
        startTime = time(NULL);
        timerWorking = 1;
        return -1;
    } else {
        if (timerWorking) {
            endTime = time(NULL);
            timerWorking = 0;
            return (double) (endTime - startTime);
        } else {
            startTime = time(NULL);
            timerWorking = 1;
            return -1;
        }
    }
}

void saveSVM(DataSet ds, const char * file){
    std::ofstream out(file);

    out << ds.m_numSamples << " " << ds.m_numFeatures << " " << ds.m_numClasses << " 1" << std::endl;
    for(Sample sample : ds.m_samples) {
        out << sample.y;
        for(int i = 0; i < ds.m_numFeatures; i++){
            out << " " << i << ":" << sample.x[i];
        }
        out << std::endl;
    }

}

void Markov::randomforest(){
    qDebug() << "Random forest";

    boost::shared_ptr<PointCloud> cloud = core_->cl_->active_;
    if(cloud == nullptr)
        return;


    // get normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = ne_->getNormals(cl_->active_);

    // zip and downsample
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr smallcloud = zipNormals(cl_->active_, normals);
    std::vector<int> big_to_small;
    smallcloud = octreeDownsample(smallcloud.get(), 0.05, big_to_small);

    // Downsample
    //std::vector<int> big_to_small;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr smallcloud = octreeDownsample(cloud.get(), 0.02, big_to_small);

    /// Setup features

    // PCA
    boost::shared_ptr<std::vector<Eigen::Vector3f> > pca = getPCA(smallcloud.get(), 0.2f, 0);

    /// Random forest

    // Hyperparameters
    Hyperparameters hp;
    hp.maxDepth = 10;
    hp.numRandomTests = 10;
    hp.numProjectionFeatures = 2;
    hp.counterThreshold = 140;
    hp.numTrees = 100;
    hp.numEpochs = 5;
    hp.useSoftVoting = 1;
    hp.verbose = 1;

    // Load selection
    std::vector<boost::shared_ptr<std::vector<int>>> selections = cloud->getSelections();

    if(selections[0]->size() < 30 || selections[1]->size() < 30) {
        qDebug() << "Not enough data";
        return;
    }

    // Creating the train data
    DataSet dataset_train, dataset_test;

    //dataset_train.m_numSamples = selections[0].size() + selections[1].size();
    dataset_train.m_numFeatures = 10;
    dataset_train.m_numClasses = 2;

    dataset_test.m_numFeatures = 10;
    dataset_test.m_numClasses = 2;

    std::set<int> seen;

    int count = 0;
    for(uint y = 0; y < selections.size(); y++){
        for(int big_idx : *selections[y]) {
            wsvector<double> x(dataset_train.m_numFeatures);
            Sample sample;
            resize(sample.x, dataset_train.m_numFeatures);

            sample.w = 1.0;
            sample.y = y;

            int idx = big_to_small[big_idx];
            if(!seen.insert(idx).second)
                continue;

            //set samples
            x[0] = smallcloud->at(idx).x;
            x[1] = smallcloud->at(idx).y;
            x[2] = smallcloud->at(idx).z;
            x[3] = smallcloud->at(idx).intensity;
            x[4] = smallcloud->at(idx).normal_x;
            x[5] = smallcloud->at(idx).normal_y;
            x[6] = smallcloud->at(idx).normal_z;
            x[7] = (*pca)[idx][0];
            x[8] = (*pca)[idx][1];
            x[9] = (*pca)[idx][2];

            copy(x, sample.x);

            if((count++%2==0))
                dataset_train.m_samples.push_back(sample);
            else
                dataset_test.m_samples.push_back(sample);
        }

    }

    dataset_test.m_numSamples = dataset_test.m_samples.size();
    dataset_train.m_numSamples = dataset_train.m_samples.size();


    // Write out to file
    saveSVM(dataset_test, "test.svm");
    saveSVM(dataset_test, "train.svm");

    qDebug() << "Size tr:" << dataset_train.m_numSamples;
    qDebug() << "Size ts:" << dataset_test.m_numSamples;

    if(dataset_train.m_numSamples < 10 || dataset_test.m_numSamples < 10){
        qDebug() << "Not enough samples";
        return;
    }


    dataset_train.findFeatRange();
    dataset_test.findFeatRange();

    OnlineRF model(hp, dataset_train.m_numClasses, dataset_train.m_numFeatures, dataset_train.m_minFeatRange, dataset_train.m_maxFeatRange);

    for(int i = 0; i < dataset_train.m_numFeatures; i++){
        qDebug() << "Min" << dataset_train.m_minFeatRange[i] << "Max" << dataset_train.m_maxFeatRange[i];
    }


    timeIt(1);
    model.trainAndTest(dataset_train, dataset_test);
    cout << "Training/Test time: " << timeIt(0) << endl;

    // fill in datasets

    // Inference here!
    // for all the other points

    Result invalid;
    //invalid.confidence.at(0) = 0;
    invalid.prediction = -1;

    std::vector<Result> results(smallcloud->points.size(), invalid);

    for(size_t idx = 0; idx < smallcloud->points.size(); ++idx) {
        wsvector<double> x(dataset_train.m_numFeatures);
        Sample sample;
        resize(sample.x, dataset_train.m_numFeatures);
        sample.w = 1.0;

        //set samples
        x[0] = smallcloud->at(idx).x;
        x[1] = smallcloud->at(idx).y;
        x[2] = smallcloud->at(idx).z;
        x[3] = smallcloud->at(idx).intensity;
        x[4] = smallcloud->at(idx).normal_x;
        x[5] = smallcloud->at(idx).normal_y;
        x[6] = smallcloud->at(idx).normal_z;
        x[7] = (*pca)[idx][0];
        x[8] = (*pca)[idx][1];
        x[9] = (*pca)[idx][2];

        copy(x, sample.x);

        results[idx] = model.eval(sample);

        if(idx % 10000 == 0){
            //for(int i = 0; i < 10; i++){
            //    qDebug() << "Sample " << i << ": " << sample.x[i];
            //}

            qDebug() << "Label" << results[idx].prediction << "Confidence: " << results[idx].confidence.at(0) << results[idx].confidence.at(1);

        }

    }

    // Select fg and bg

    auto fgselect = boost::make_shared<std::vector<int>>();
    auto bgselect = boost::make_shared<std::vector<int>>();

    for(size_t idx = 0; idx < cloud->points.size(); ++idx) {
        int idx_small = big_to_small[idx];
        Result & res = results[idx_small];

        if(res.prediction == 0)
            fgselect->push_back(idx);
        else
            bgselect->push_back(idx);
    }

    Select * fgselectcmd = new Select(cl_->active_, fgselect, nullptr, 1);
    Select * bgselectcmd = new Select(cl_->active_, bgselect, nullptr, 2);

    core_->us_->beginMacro("Random forest");
    core_->us_->push(fgselectcmd);
    core_->us_->push(bgselectcmd);
    core_->us_->endMacro();

/*
    // Determine veg
    std::vector<bool> likely_veg(smaller_cloud->size());

    for(uint i = 0; i < pca->size(); i++) {
        Eigen::Vector3f eig = (*pca)[i];

        // If not enough neighbours
        if(eig[1] < eig[2]) {
            likely_veg[i] = false;
            continue;
        }

        float eig_sum = eig.sum();

        eig /= eig_sum;

        float fudge_factor = 5.0f;
        if(eig[1] < 0.05 * fudge_factor || eig[2] < 0.01 * fudge_factor) {
            likely_veg[i] = false;
        } else {
            likely_veg[i] = true;
        }

    }
*/

/*
    std::set<int> foreground_points;

    // Map veg to foreground points in 2nd downsampled cloud
    for(uint i = 0; i < cloud->size(); i++) {
        uint pca_idx = big_to_small_map[i];
        if(likely_veg[pca_idx] == true){
            uint small_idx = big_to_small_map[i];
            foreground_points.insert(small_idx);
        }
    }


    // Set up graph cut

    MinCut mc;
    mc.setInputCloud(smaller_cloud);


    //foreground_points.push_back(big_to_small_map[fg_idx_]);

    //std::vector<int> background_points;
    //background_points.push_back(big_to_small_map[idx]);

    double radius = 3.0;
    double sigma = 0.25;
    int neigbours = 10;
    double source_weight = 0.8;

    mc.setForegroundPoints (foreground_points);
    //mc.setBackgroundPoints(background_points);
    mc.setRadius (radius);
    mc.setSigma (sigma);
    mc.setNumberOfNeighbours (neigbours);
    mc.setSourceWeight (source_weight);

    std::vector<pcl::PointIndices> clusters;
    mc.extract (clusters);

    auto select = boost::make_shared<std::vector<int>>();

    // GAH!!!! inefficient

    std::vector<bool> small_idxs_selected(smaller_cloud->size(), false);
    for(int idx : clusters[1].indices){
        small_idxs_selected[idx] = true;
    }

    for(size_t i = 0; i < big_to_small_map.size(); i++) {
        int idx = big_to_small_map[i];
        if(small_idxs_selected[idx]) {
            select->push_back(i);
        }
    }

    //std::copy(clusters[1].indices.begin(), clusters[1].indices.end(), select->begin());

    Select * selectCmd = new Select(cl_->active_, select);



    core_->us_->beginMacro("Markov min cut");
    core_->us_->push(selectCmd);
    core_->us_->endMacro();
    core_->cl_->updated();
    core_->mw_->stopBgAction("Cut completed.");


    // We need a layer for the region of interest

    // Need a layer for the pixels pinned to the fg

    // Need a layer for the pixels pinned to the bg
    fg_idx_ = -1;
    disable();
    */
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
