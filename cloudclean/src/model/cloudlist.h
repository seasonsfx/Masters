#ifndef CLOUDLIST_H
#define CLOUDLIST_H

#include <vector>
#include <QAbstractListModel>
#include <model/pointcloud.h>

class CloudList : public QAbstractListModel {
    Q_OBJECT
 public:
    explicit CloudList(QObject *parent = 0);
    int rowCount(const QModelIndex & parent = QModelIndex()) const;
    QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const;
    std::shared_ptr<PointCloud> addCloud();
    std::shared_ptr<PointCloud> addCloud(const char* filename);
    std::shared_ptr<PointCloud> addCloud(std::shared_ptr<PointCloud> pc);
    
 signals:
    void cloudUpdate(int id);

 public slots:
    
 public:
    std::vector<std::shared_ptr<PointCloud> > clouds_;
};

#endif // CLOUDLIST_H
