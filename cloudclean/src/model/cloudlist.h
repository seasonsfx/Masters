#ifndef CLOUDLIST_H
#define CLOUDLIST_H

#include <vector>
#include <mutex>
#include <memory>
#include <QAbstractListModel>
#include <model/pointcloud.h>

class QUndoStack;
class QItemSelection;

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
    void removeCloud(int idx);
    
 signals:
    void cloudUpdate(std::shared_ptr<PointCloud> pc);
    void updated(); // Very vague
    void changedSelection(std::vector<int> selection);
    void updatedActive(std::shared_ptr<PointCloud> pc);
    void deletingCloud(std::shared_ptr<PointCloud> cloud);

    void progressUpdate(int percentage);
    void startNonDetJob();
    void endNonDetJob();

 public slots:
    void removeCloud();
    void selectionChanged(const QItemSelection &sel, const QItemSelection &des);
    std::shared_ptr<PointCloud> loadFile(QString filename);
    
 private:
    std::mutex * mtx_;

 public:
    std::vector<std::shared_ptr<PointCloud> > clouds_;
    std::vector<int> selection_;
    std::shared_ptr<PointCloud> active_;
};

#endif // CLOUDLIST_H
