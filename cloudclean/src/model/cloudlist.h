#ifndef CLOUDLIST_H
#define CLOUDLIST_H

#include <QAbstractListModel>

class CloudList : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit CloudList(QObject *parent = 0);
    
signals:
    
public slots:
    
};

#endif // CLOUDLIST_H
