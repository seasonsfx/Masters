#ifndef CLOUDCLEAN_H
#define CLOUDCLEAN_H

#include <QWidget>
#include <QModelIndex>
//#include "glwidget.h"

namespace Ui {
    class CloudClean;
}

class CloudClean : public QWidget
{
    Q_OBJECT

public:
    explicit CloudClean(QWidget *parent = 0);
    ~CloudClean();

private:
    Ui::CloudClean *ui;
    //GLWidget * glwidget;

public slots:
    void clickedLayer(const QModelIndex & index);
	bool loadScan();
    bool saveScan();

signals:
	void reloadCloud();

};


	

#endif // CLOUDCLEAN_H
