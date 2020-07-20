/***************************************************************************
 *                                                                         *
 *
 * author:Simón C. Smith
 * e-mail:artificialsimon@gmail.com
 *
 ***************************************************************************/

#ifndef __DIAMOND_H
#define __DIAMOND_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include "abstractdiamondcontroller.h"

// base controllers
#include <selforg/sox.h>
#include "soxdiamond.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/teachable.h>
#include <selforg/parametrizable.h>

// utils
#include <string>
#include <vector>
using namespace std;

/// configuration object for Diamond controller. Use Diamond::getDefaultConf().
struct DiamondConf {
  int    n_layers;  ///< number of internal layers
  string base_controller_name;
  bool someInternalParams;
};


/**
 * This controller implements the standard algorihm described the the Chapter 5 (Homeokinesis)
 *  with extensions of Chapter 15 of book "The Playful Machine"
 */
class Diamond : public AbstractController, public Teachable, public Parametrizable {

public:
  Sox *ol;
  Sox oll;
  /// constructor
  Diamond(const DiamondConf& conf = getDefaultConf());

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~Diamond();

  static DiamondConf getDefaultConf(){
    DiamondConf conf;
    conf.n_layers = 2;
    conf.base_controller_name = "Sox";
    conf.someInternalParams = true;
    return conf;
  }


  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /* some direct access functions (unsafe!) */
  virtual matrix::Matrix getA();
  virtual void setA(const matrix::Matrix& A);
  virtual matrix::Matrix getC();
  virtual void setC(const matrix::Matrix& C);
  virtual matrix::Matrix geth();
  virtual void seth(const matrix::Matrix& h);

  /***** TEACHABLE ****/
  virtual void setMotorTeaching(const matrix::Matrix& teaching);
  virtual void setSensorTeaching(const matrix::Matrix& teaching);
  virtual matrix::Matrix getLastMotorValues();
  virtual matrix::Matrix getLastSensorValues();

  /***** PARAMETRIZABLE ****/
  virtual std::list<matrix::Matrix> getParameters() const override;
  virtual int setParameters(const std::list<matrix::Matrix>& params) override;

protected:
  vector<SoxDiamond*> internal_layer; // internal layer controller
  SoxDiamond* make_layer(string base_controller_name); // Factory for base c.
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 10;

  matrix::Matrix A; // Model Matrix
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix S; // Model Matrix (sensor branch)
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix
  matrix::Matrix R; //
  matrix::Matrix C_native; // Controller Matrix obtained from motor babbling
  matrix::Matrix A_native; // Model Matrix obtained from motor babbling
  matrix::Matrix y_buffer[buffersize]; // buffer needed for delay
  matrix::Matrix x_buffer[buffersize]; // buffer of sensor values
  matrix::Matrix v_avg;
  matrix::Matrix x;        // current sensor value vector
  matrix::Matrix x_smooth; // time average of x values
  int t;

  bool loga;

  DiamondConf conf; ///< configuration objects

  bool intern_isTeaching;    // teaching signal available?
  matrix::Matrix y_teaching; // motor teaching  signal

  paramval creativity;
  paramval sense;
  paramval harmony;
  paramval causeaware;
  paramint pseudo;
  paramval epsC;
  paramval epsA;
  paramval damping;
  paramval gamma;          // teaching strength

  void constructor();

  // calculates the pseudo inverse of L in different ways, depending on pseudo
  matrix::Matrix pseudoInvL(const matrix::Matrix& L, const matrix::Matrix& A, const matrix::Matrix& C);

  /// learn values model and controller (A,b,C,h)
  virtual void learn();

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

  /// derivative of g
  static double g_s(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
  };

  /// inverse of neuron activation
  static double g_inv(double z)
  {
    return atanh(clip(.99, z));
  };


  /// function that clips the second argument to the interval [-first,first]
  static double clip(double r, double x){
    return min(max(x,-r),r);
  }
  /// calculates the inverse the argument (useful for Matrix::map)
  static double one_over(double x){
    return 1/x;
  }


};

#endif


