#ifndef MBED_SERIALPLOT_HPP
#define MBED_SERIALPLOT_HPP

#include <vector>
#include "mbed.h"

#define FlagDebug 0x0A

class SerialPlot {
public:
    SerialPlot(BufferedSerial *pc, EventFlags *flag);

    ~SerialPlot();

/**
 * Ajout du pointeur vers la variables de type `int` à la liste de variable à afficher.
 * @param ptrVariable : float *
 */
    void addVariables(float *ptrVariable);


    /**
     * Méthode lançant le thread de Communication dit thread de débogage
     */
    void run();


private:
/**
 * Port série à utiliser pour le débogage
 */
    BufferedSerial *_pc;

    /**
     * Flag permettant le cadencement de l'affichage.
     * La fonction `Worker()` fonctionne dès que le flag est levé.
     */
    EventFlags *_flag;

    /**
     * Méthode privée permettant l'affichage des valeurs des pointeurs contenu dans les listes de variables.
     * En commençant par les flottantes, puis les int et enfin les uint16_t
     */
    void Worker();

    /**
     * Thread de Communication utilisé pour le débogage par la communication série
     */
    Thread debug_thread;

    /**
     * Fonction d'éxécution du thread, fait fonctionner `Worker()` au rythme du flag
     */
    void threadWorker();

    /**
     * Listes des pointeur vers les variables flottantes
     */
    vector<float *> floatVariables;
};


#endif //MBED_SERIALPLOT_HPP
