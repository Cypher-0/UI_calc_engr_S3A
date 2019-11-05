#ifndef REDUCER_H
#define REDUCER_H

#include <QObject>

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

public slots:
    void calcPower(const double &ImotorPwr, const double &ImotorSpeed, const double &IloadMass, const double &IsafetyCoef,
                   const double &IloadHeight, const double &IcableDiam, const double &IspoolWidth, const double &IspoolDiam);

private :

    ///////////////////////////////////////////
    //
    //          Variables
    //
    ///////////////////////////////////////////

    //Power
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

    ////// Misc

    //input parameters
    double gearsEfficiency = 0.95;

    //ouput parameters
    double totalReduc = 0.0;


    ///////////////////////////////////////////
    //
    //          Fonctions
    //
    ///////////////////////////////////////////

    double calcMaxSpoolRad();
};

#endif // REDUCER_H
