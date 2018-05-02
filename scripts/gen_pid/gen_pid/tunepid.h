#ifndef TUNEPID_H
#define TUNEPID_H

#include <QMainWindow>

namespace Ui {
class TunePID;
}

class TunePID : public QMainWindow
{
    Q_OBJECT

public:
    explicit TunePID(QWidget *parent = 0);
    ~TunePID();

private:
    Ui::TunePID *ui;
};

#endif // TUNEPID_H
