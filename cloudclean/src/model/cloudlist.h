#ifndef CLOUDLIST_H
#define CLOUDLIST_H

#include <vector>
#include <mutex>
#include <QAbstractListModel>
#include <model/pointcloud.h>

class CloudList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit CloudList(QObject *parent = 0);
    ~CloudList();
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    std::shared_ptr<PointCloud> addCloud();
    std::shared_ptr<PointCloud> addCloud(const char* filename);
    std::shared_ptr<PointCloud> addCloud(std::shared_ptr<PointCloud> pc);
    
 signals:
    void cloudUpdate(std::shared_ptr<PointCloud> pc);
    void updated();

 public slots:
    
 private:
    std::mutex * mtx_;

 public:
    std::vector<std::shared_ptr<PointCloud> > clouds_;
};

#endif // CLOUDLIST_H
