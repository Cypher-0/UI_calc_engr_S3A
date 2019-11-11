#ifndef REDUCER_H
#define REDUCER_H

//définition d'un macro globale pour l'extension de fichier
#define FILE_TYPE QString(".cgp")

#include <QObject>
#include <QVector>
#include <math.h>

class Reducer : public QObject
{
    Q_OBJECT

public:
    explicit Reducer(QObject *parent = nullptr);

    ///////////////////////////////////////////
    //
    //          Accesseurs
    //
    ///////////////////////////////////////////

        //////////////////////////////////
        ///         GEARS
        //////////////////////////////////

    /////input parameters

    double get_loadHeight();
    double get_loadMass();
    double get_motorPwr();
    double get_motorSpeed();
    double get_cableDiam();
    double get_spoolDiam();
    double get_spoolWidth();

    /////results
        //lower shaft

    double get_lSInputFreq();
    double get_lSOutputFreq();
    double get_lSPwr();
    double get_lSRatedTorque();
    double get_lSReducRatio();

    //motor shaft

    double get_mSInputFreq();
    double get_mSOutputFreq();
    double get_mSPwr();
    double get_mSRatedTorque();
    double get_mSReducRatio();

    //winch

    double get_wInputFreq();
    double get_wOutputFreq();
    double get_wPwr();
    double get_wRatedTorque();
    double get_wReducRatio();

    //misc

    double get_calcLoadMass();
    double get_loadWeight();
    double get_maxSpoolRad();
    double get_safetyCoef();

        //////////////////////////////////
        ///         GLOBAL
        //////////////////////////////////

    double get_totalReduc();

signals:
    ////// GEARS
    void update_PB_gearsCalcRange(int min,int max);
    void update_PB_gearsCalcValue(int value);

    void actGearsResult(int Z1,int Z2,int Z3,int Z4,double m1,double m2,
                        double b12,double b34,double calculatedReducRatio,
                        double,double,double,double);
    void gearsCalcEnded();


    ////// VERIF GEARS
    void actVerifGearsExpectedValues(double alignTolerance,double reducRatio);
    void actVerifGearsGotValues(double alignTolerance,double reducRatio,bool alignToleranceOK,bool reducRatioOk);

public slots:
    ////// POWER
    void calcPower(const double &ImotorPwr, const double &ImotorSpeed, const double &IloadMass, const double &IsafetyCoef,
                   const double &IloadHeight, const double &IcableDiam, const double &IspoolWidth, const double &IspoolDiam);



    ////// GEARS
    void calcGears();
    void actGearsInput(const int &IZmin, const int &IZmax, const double &ImMin, const double &ImMax, const double &ImStep,
                                        const int &Irpe, const double &IgearsAlignTolerance, const double &IaxisDiam, bool IconsiderAlignTolerance);

    ////// VERIF GEARS
    void actVerifGearsInput(const int &IZ1,const int &IZ2,const int &IZ3,const int &IZ4);

    ////// GLOBAL
    void saveProjectOutput(QString saveName);
    void loadProjectOutput(QString saveName);

private :

    ///////////////////////////////////////////
    //
    //          Variables
    //
    ///////////////////////////////////////////

    ////// POWER

        //input
    double loadHeight = 0.0;
    double safetyCoef = 0.0;
    double loadMass = 0.0;
    double motorPwr = 0.0;
    double motorSpeed = 0.0;
    double cableDiam = 0.0;
    double spoolDiam = 0.0;
    double spoolWidth = 0.0;
        //result
    /* lS = lowerShaft (arbre inférieur)
     * mS = motorShaft (arbre moteur)
     * w = winch (treuil)
    */
    double lSInputFreq = 0.0;
    double lSOutputFreq = 0.0;
    double lSPwr = 0.0;
    double lSRatedTorque = 0.0;
    double lSReducRatio = 0.0;

    double mSInputFreq = 0.0;
    double mSOutputFreq = 0.0;
    double mSPwr = 0.0;
    double mSRatedTorque = 0.0;
    double mSReducRatio = 0.0;

    double wInputFreq = 0.0;
    double wOutputFreq = 0.0;
    double wPwr = 0.0;
    double wRatedTorque = 0.0;
    double wReducRatio = 0.0;

    double calcLoadMass = 0.0;
    double loadWeight = 0.0;
    double maxSpoolRad = 0.0;

    ////// GEARS

    QVector<double> klist;

    //calculated values
    int bestZ1=999990,bestZ2=999990,bestZ3=999990,bestZ4=999990;
    double bestm1=0.0,bestm2=0.0;
    double bestk1=0.0,bestk2=0.0;
    double bestR1=999.0,bestR2=999.0,bestR3=999.0,bestR4=999.0;
    double calculatedReducRatio = 0.0;

        //input
    double mMin=0.0,mMax=0.0,mStep=0.0,gearsAlignTolerance=0.0;
    int Zmin=0.0,Zmax=0.0,rpe=0;
    double alpha = 20.0;//angle de pression (°)
    double axisDiam = 0.0;
    double wedgeSizeSafety = 5.0;
    bool considerAlignTolerance = false;

        //calc values
    double Z1=0.0,Z2=0.0,Z3=0.0,Z4=0.0;
    int tmpZ1=0,tmpZ2=0,tmpZ3=0,tmpZ4=0;
    double m1=0.0,m2=0.0;
    double r1=0.0,r2=0.0,r3=0.0,r4=0.0;

    ////// VERIF GEARS
    double veGears_Z1=0.0,veGears_Z2=0.0,veGears_Z3=0.0,veGears_Z4=0.0;

    ////// Misc

    //input parameters
    double gearsEfficiency = 0.95;

    //output parameters
    double totalReduc = 0.0;


    ///////////////////////////////////////////
    //
    //          Fonctions
    //
    ///////////////////////////////////////////

    //POWER
    double calcMaxSpoolRad();

    //GEARS

};

inline static void adjustValues(double &Z1,double &Z2,double &Z3,double &Z4)
{
    Z1 = double(round(Z1));
    Z2 = double(round(Z2));
    Z3 = double(round(Z3));
    Z4 = double(round(Z4));
}

double getWedgeHeight(const double &axisDiam);//obtenir la taille de la clavette en fonction du diamètre de l'arbre

//file
QString getStrValueOf(const QString &input,const QString &param);

#endif // REDUCER_H
