#ifndef CLOUDCLEAN_H
#define CLOUDCLEAN_H

#include <QWidget>

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

public slots:
	bool loadScan();

signals:
	void reloadCloud();

};


	

#endif // CLOUDCLEAN_H
