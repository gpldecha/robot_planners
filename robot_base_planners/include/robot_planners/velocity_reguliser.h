#ifndef VELOCITY_REGULISER_H_
#define VELOCITY_REGULISER_H_

class Velocity_reguliser{

public:

    Velocity_reguliser(double max_speed=0.1, double min_speed=0);

    /**
     * @brief bel_shape_curve: speed is modulated as a function of the distance to the
     *                         target.
     * @param distance_target: distance to target point (meters).
     * @param beta           : stifness (1/var), the higher the stiffness,
     *                         the higher the velocity
     * @return               : speed amplitude to be multiplied to direction vector
     */
    double bel_shape_curve(double distance_target_m,double beta);

    void set_min_speed_ms(double min_speed);

    void set_max_speed_ms(double max_speed);

private:

    inline double scale(double x, double min_v, double max_v, double a, double b){
        return ((b-a) * (x - min_v))/(max_v - min_v) + a;
    }

private:

    double dt;
    double min_speed;
    double max_speed;
    double distance_target_cm;

};

#endif
