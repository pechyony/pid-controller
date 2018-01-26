#ifndef PID_H
#define PID_H

// PID controller: -Kp*cte(t) - Ki*(\sum_{i=0}^t cte(t)) - Kd*(cte(t)-cte(t-1)) 
class PID {
public:
  /**
  * Errors
  */
  double p_error;   // cte(t)
  double i_error;   // \sum_{i=0}^t cte(t)
  double d_error;   // cte(t)-cte(t-1)

  /**
  * Parameters
  */ 
  double Kp;
  double Ki;
  double Kd;

  /**
  * Constructor
  */
  PID();

  /**
  * Destructor.
  */
  virtual ~PID();

  /**
  * init Initialize controller
  * @param Kp - value of Kp parameter
  * @param Ki - value of Ki parameter
  * @param Kd - value of Kd parameter
  */
  void Init(double Kp, double Ki, double Kd);

  /**
  * UpdateError Update parameters of PID controller given cross track error
  * @param cte - cross-track error
  */
  void UpdateError(double cte);

  /**
  * TotalError Calculate the total error of PID controller
  */
  double TotalError();
};

// PIDAbs controller: -Kp*|cte(t)| - Ki*|\sum_{i=0}^t cte(t)| - Kd*(cte(t)-cte(t-1)) 
class PIDAbs : public PID {
public: 
	/**
	* TotalError Calculate the total error of PIDAbs controller.
	*/
	double TotalError();
};

#endif /* PID_H */
