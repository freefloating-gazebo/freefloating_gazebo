#include "tunepid.h"
#include "ui_tunepid.h"

TunePID::TunePID(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TunePID)
{
    ui->setupUi(this);
}

TunePID::~TunePID()
{
    delete ui;
}
