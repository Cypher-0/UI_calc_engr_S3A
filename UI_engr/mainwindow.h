#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "reducer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots :
    void actPwrResult(int useless);
    void actPwrResult(double useless);

    void actGearsParams(int useless);
    void actGearsParams(double useless);

private slots:
    void on_pb_startGearsCalc_clicked();

private:
    Ui::MainWindow *ui;

    Reducer reduc;
};
#endif // MAINWINDOW_H
