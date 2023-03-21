#ifndef brushless_hpp
#define brushless_hpp

#include <cmath>
#include "mbed.h"
#include "tim.h"
#include "Controllers/Controller.h"
#include "Controllers/PIController.h"
#include "Controllers/PIDController.h"

#define MAX_SPEED            20000
#define MIN_SPEED            (-20000)
#define DutyCylcleMAX        90
#define ticksPerRevolution   (14*48)

typedef struct {
    bool aH;
    bool aL;
    bool bH;
    bool bL;
    bool cH;
    bool cL;
} halfBridge_t;

/**  The following Table  describes the TIM1 Channels states (counterclockwise):
  -----------------------------------------------------------
  |Hall 1/2/3|  101  |  100  |  110  |  010  |  011  |  001  |
   ----------------------------------------------------------
  | aH-Ch1   |   0   |   0   |   0   |   1   |   1   |   0   |
   --------  --------------------------------------------------
  | aL-Ch1N  |   1   |   1   |   0   |   0   |   0   |   0   |
   --------  --------------------------------------------------
  | bH-Ch2   |   1   |   0   |   0   |   0   |   0   |   1   |
   --------  --------------------------------------------------
  | bL-Ch2N  |   0   |   0   |   1   |   1   |   0   |   0   |
   --------  --------------------------------------------------
  | cH-Ch3   |   0   |   1   |   1   |   0   |   0   |   0   |
   --------  --------------------------------------------------
  | cL-Ch3N  |   0   |   0   |   0   |   0   |   1   |   1   |
   ----------------------------------------------------------

   001 -> 011 -> 010 -> 110 -> 100 -> 101
*/
const halfBridge_t clockwiseSequence[6] = {
        {0,0,1,0,0,1}, // 0b001
        {1,0,0,1,0,0}, // 0b010
        {1,0,0,0,0,1}, // 0b011
        {0,1,0,0,1,0}, // 0b100
        {0,1,1,0,0,0}, // 0b101
        {0,0,0,1,1,0}, // 0b110
};
/**  The following Table  describes the TIM1 Channels states (clockwise):
  -----------------------------------------------------------
  |Hall 1/2/3|  101  |  100  |  110  |  010  |  011  |  001  |
   ----------------------------------------------------------
  | aH-Ch1   |   1   |   1   |   0   |   0   |   0   |   0   |
   --------  --------------------------------------------------
  | aL-Ch1N  |   0   |   0   |   0   |   1   |   1   |   0   |
   --------  --------------------------------------------------
  | bH-Ch2   |   0   |   0   |   1   |   1   |   0   |   0   |
   --------  --------------------------------------------------
  | bL-Ch2N  |   1   |   0   |   0   |   0   |   0   |   1   |
   --------  --------------------------------------------------
  | cH-Ch3   |   0   |   0   |   0   |   0   |   1   |   1   |
   --------  --------------------------------------------------
  | cL-Ch3N  |   0   |   1   |   1   |   0   |   0   |   0   |
   ----------------------------------------------------------

   101 -> 100 -> 110 -> 010 -> 011 -> 001
*/
const halfBridge_t antiClockwiseSequence[6] = {
        {0,0,0,1,1,0}, // 0b001
        {0,1,1,0,0,0}, // 0b010
        {0,1,0,0,1,0}, // 0b011
        {1,0,0,0,0,1}, // 0b100
        {1,0,0,1,0,0}, // 0b101
        {0,0,1,0,0,1}, // 0b110
};

enum rotationSens_t {
    clockwise, antiClockwise
};
enum state {
    activated, desactivated, emergency
};
enum position {
    Left, Right
};
enum unitVelocity {
    rad_s, tick_s, mm_s
};

typedef struct {
    bool h1;
    bool h2;
    bool h3;
    char h123;
    char prev_h123;    //to determine sense of rotation

    int32_t ticks;    //to calculate ticks
} hall_t;

class BrushlessEirbot {
public:
    /**
     * Constructeur de l'object BrushlessEirbot avec l'information de la position du moteur. La position détermine le timer spécial utilisé.
     * Ils sont au nombre de 2 (timer1 et timer8)
     * Si la position est define sur *Left* le timer utilisé est le timer 1.
     * Si la position est define sur *Right* le timer utilisé est le timer 8.
     *
     *
     * @param position : Left ou Right selon le timer utilisé
     * @param wheelDiameterMm : Diamètre de la roue en millimètre
     */
    BrushlessEirbot(position position_motor, float wheelDiameterMm);

    /**
     * Constructeur permettant de renseigner un port série de débug.
     * @param pc : Port série de debug
     * @param position_motor : Left ou Right selon le timer utilisé
     * @param wheelDiameterMm : Diamètre de la roue en millimètre
     */
    BrushlessEirbot(BufferedSerial *pc, position position_motor, float wheelDiameterMm);

    ~BrushlessEirbot();

    /**
     * Implémentation d'un correcteur PI
     * @param Kp
     * @param wi
     * @param Te
     */
    void setPI(float Kp, float wi, std::chrono::microseconds Te = 10ms);

    /**
     * Implémentation d'un correcteur PID
     * @param Kp
     * @param wi
     * @param wb
     * @param wh
     * @param TeUsController
     */
    void setPID(float Kp, float wi, float wb, float wh, std::chrono::microseconds TeUsController = 10ms);

    /**
     * Consigne en vitesse
     * @param unit
     * @param consigne
     */
    void setVelocity(unitVelocity unit, float consigne);

    /**
     * Décide de l'état de l'asservissement
     * @param stateController :
     */
    void setController(state stateController);

    /**
     * Revoie la valeur de la vitesse actuelle du robot.
     * Cette vitesse est relevée toutes les 10 ms. (pour l'assevissement)
     * @param unit : Soit en rad_s (radian par seconde), tick_s (ticks par seconde), mm_s (millimètre par seconde)
     * @return double velocity
     */
    float getVelocity(unitVelocity unit) const;

    /**
    * Affiche sur le port de debug le pinOut
    * Faisable seulement si le port de debug à été configuré
    */
    void displayPinOut();

    /**
 * Met à jour les commutation sur les demi-ponts
 */
    void updateOutput();

// Méthodes d'asservissements
    /**
    * Configure le rapport cyclique pour régler l'intensité du champs magnétique du stator sur le rotor.
    * Fait à 100Hz dans le code de Vincent (nommée writeCommande)
    */
    void setDutyCycle(float dutyCycle);


private:
    state _stateController;
    Timer _timerVelocity;
    position _positionMotor;
    float _wheelDiameterMm;
    rotationSens_t _sens;
    int32_t _ticks;
    BufferedSerial *_serial{0};
    bool _debug{false};

    hall_t hall{0};
// Méthodes primitives
    /**
     * Décode les interruptions sur les capteurs à effet Hall pour prévoir la prochaine séquence de commutation des demi-ponts.
     * Compte les ticks.
     * Fait à 100 kHz dans le code de Vincent
     */
    void decodeHall();


    /**
     * Calcul la vitesse de l'arbre moteur
     * Fait à 100Hz dans le code de Vincent
     * @return
     */
    int16_t calculateSpeed();

// Pins et instances
    PinName _pinPWM_AH;
    PinName _pinPWM_AL;
    PinName _pinPWM_BH;
    PinName _pinPWM_BL;
    PinName _pinPWM_CH;
    PinName _pinPWM_CL;
    PinName _pinHall_1;
    PinName _pinHall_2;
    PinName _pinHall_3;
    PinName _pinCurrent_A;
    PinName _pinCurrent_B;
    PinName _pinCurrent_C;

    InterruptIn *HALL_1;
    InterruptIn *HALL_2;
    InterruptIn *HALL_3;

    void hallInterrupt();

    AnalogIn *Current_A;
    AnalogIn *Current_B;
    AnalogIn *Current_C;

    Controller* controller;

    std::chrono::microseconds TeUsController = 10ms;
    Ticker _tickerController;
    /**
     * Calcul la vitesse et gère l'asservissement de la vitesse du moteur
     */
    void _routineController();
};

#endif