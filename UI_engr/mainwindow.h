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

    void update_PB_gearsCalcRange(int min,int max);
    void actGearsResult(int Z1,int Z2,int Z3,int Z4,double m1,double m2,double minM1,double minM2,
                        double b12,double b34,double calculatedReducRatio,
                        double r1,double r2,double r3, double r4);
    void gearsCalcEnded();

    void actVerifGearsExpectedValues(double alignTolerance,double reducRatio);
    void actVerifGearsGotValues(double alignTolerance,double reducRatio,bool alignToleranceOK,bool reducRatioOk);
    void updateVerifGearsInput(int useless);

    //Bearings
    void actBearingsInput(double useless);
    void actBearingsOutput();


    //make saves
    void saveProjectInput(QString saveName);
    void loadProjectInput(QString saveName);
    void saveAllProject(QString fileName);
    void loadAllProject(QString fileName);

private slots:
    void on_pb_startGearsCalc_clicked();

    void on_actionSauvegarder_le_projet_triggered();

    void on_actionCharger_le_projet_triggered();

private:
    Ui::MainWindow *ui;

    Reducer reduc;
};
#endif // MAINWINDOW_H
