/*
        Lander Control simulation.

        Updated by F. Estrada for CSC C85, Oct. 2013
        Updated by Per Parker, Sep. 2015

        Learning goals:

        - To explore the implementation of control software
          that is robust to malfunctions/failures.

        The exercise:

        - The program loads a terrain map from a .ppm file.
          the map shows a red platform which is the location
          a landing module should arrive at.
        - The control software has to navigate the lander
          to this location and deposit the lander on the
          ground considering:

          * Maximum vertical speed should be less than 10 m/s at touchdown
          * Maximum landing Robust(Angle) should be less than 15 degrees w.r.t vertical

        - Of course, touching any part of the terrain except
          for the landing platform will result in destruction
          of the lander

        This has been made into many videogames. The oldest one
        I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

        Your task:

        - These are the 'sensors' you have available to control
          the lander.

          FVelocity_X();  - Gives you the lander's horizontal velocity
          FVelocity_Y();	 - Gives you the lander's vertical velocity
          FPosition_X();  - Gives you the lander's horizontal position (0 to 1024)
          FPosition_Y();  - Gives you the lander's vertical position (0 to 1024)

          FAngle();	 - Gives the lander's Robust(Angle) w.r.t. vertical in DEGREES (upside-down = 180 degrees)

          SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to Robust(Angle) w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          FRangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

        - Variables accessible to your 'in flight' computer

          MT_OK		- Boolean, if 1 indicates the main thruster is working properly
          RT_OK		- Boolean, if 1 indicates the right thruster is working properly
          LT_OK		- Boolean, if 1 indicates the left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y  - Y position of the landing platform

        - Control of the lander is via the following functions
          (which are noisy!)

          Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
          Left_Thruster(double power);	 - Sets left thruster power in [0 1]
          Right_Thruster(double power);  - Sets right thruster power in [0 1]
          Rotate(double Robust(Angle));	 	 - Rotates module 'Robust(Angle)' degrees clockwise
                                           (ccw if Robust(Angle) is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

                                           Note that rotation takes time!


        - Important constants

          G_ACCEL = 8.87	- Gravitational acceleration on Venus
          MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
          RT_ACCEL = 25.0	- Max acceleration provided by right thruster
          LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

        - Functions you need to analyze and possibly change

          * The Lander_Control(); function, which determines where the lander should
            go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

        - You *can* add your own helper functions (e.g. write a robust thruster
          handler, or your own robust sensor functions - of course, these must
          use the noisy and possibly faulty ones!).

        - The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

                         Initial lander position, orientation, and velocity are
                         randomized.

          * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

          * Failure modes: 0 - Nothing ever fails, life is simple
                           1 - Controls can fail, sensors are always reliable
                           2 - Both controls and sensors can fail (and do!)
                           3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Robust(Angle) sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and Robust(Angle) sensor.

                * Note - while running. Pressing 'q' on the keyboard terminates the
                        program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

        Have fun! try not to crash too many landers, they are expensive!

        Credits: Lander image and rocky texture provided by NASA
                 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/

#include "Lander_Control.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#define UP_ACCEL 25
#define FLOATING_TOLERANCE 0.00001f
#define HOVER_HEIGHT 70
#define HOVER_HEIGHT 70
#define NSAMPLES 120000
#define MAIN_OFFSET 0.0
#define LEFT_OFFSET 90.0
#define RIGHT_OFFSET -90.0
#define DELIVERY_RATIO 0.95  // Rotate(), Main/Left/Right_Thruster() all deliver 95% of whatever is asked \
                             // (mean 0.94999, median 0.95000 over 292 samples of all controls; linear, no offset)
#define MAX_TILT 60.0 // degrees - never tip the firing thruster further than this from straight up
#define ROTATE_TOLERANCE 3.0 // degrees - skip re-rotating for corrections this small \
                             // (without this, any nonzero residual misalignment      \
                             // re-triggers a full rotate cycle and thrust never sustains)
#define ALIGN_ALTITUDE 25.0
#define PLAT_TOL 10.0  

/*****************************************************************************
 ******************* ROBUST SENSOR AND CONTROL FUNCTIONS *********************
 *****************************************************************************/

enum NormalizeType {
    Nangle,
    Nthrust
};

double normalize_value(double value, NormalizeType type) {
    if (type == Nangle) {
        value = fmod(value, 360.0);
        if (value > 180.0)
            value -= 360.0;
        else if (value <= -180.0)
            value += 360.0;
    } else if (type == Nthrust) {
        value = fmin(fmax(value / DELIVERY_RATIO, 0.0), 1.0);
    }
    return value;
}

double Robust(double (*Sensor)(void)) {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++) {
        if (Sensor == Angle)
            sum += normalize_value(Sensor(), Nangle);
        else
            sum += Sensor();
    }
    return sum / NSAMPLES;
}

/* Make sure value is normalized */
void Robust(void (*Control)(double), double normalized_value) {
    Control(normalized_value);
}

double find_min_travel_angle(double required_angle, double state_angle) {
    /* This function calculates the angle to make inside our call to
     * rotate from one angle to another. */

    double diff = required_angle - state_angle;

    diff = fmod(diff, 360.0);

    if (diff <= -180.0) {
        diff += 360.0;
    } else if (diff > 180.0) {
        diff -= 360.0;
    }
    return diff;
}

class TaskControl {
    double m_accel = 0.0;
    double l_accel = 0.0;
    double r_accel = 0.0;

    enum WhichThruster {
        MAIN_THRUSTER,
        LEFT_THRUSTER,
        RIGHT_THRUSTER
    };

    static double push_offset(WhichThruster t) {
        switch (t) {
            case LEFT_THRUSTER:
                return LEFT_OFFSET;
            case RIGHT_THRUSTER:
                return RIGHT_OFFSET;
            default:
                return MAIN_OFFSET;
        }
    }

    static double max_accel(WhichThruster t) {
        switch (t) {
            case LEFT_THRUSTER:
                return LT_ACCEL;
            case RIGHT_THRUSTER:
                return RT_ACCEL;
            default:
                return MT_ACCEL;
        }
    }

    static WhichThruster pick_thruster() {
        if (MT_OK) return MAIN_THRUSTER;
        if (LT_OK) return LEFT_THRUSTER;
        return RIGHT_THRUSTER;
    }

    void apply() {
        double angle = Robust(Angle);

        if (MT_OK && LT_OK && RT_OK) {
            // Nothing broken: stay upright and drive each thruster directly.
            robust_rotate(find_min_travel_angle(0.0, angle));
            Robust(Main_Thruster, normalize_value(m_accel / MT_ACCEL, Nthrust));
            Robust(Left_Thruster, normalize_value(l_accel / LT_ACCEL, Nthrust));
            Robust(Right_Thruster, normalize_value(r_accel / RT_ACCEL, Nthrust));
            return;
        }

        // Something is broken: point one working thruster along the requested vector.
        WhichThruster t = pick_thruster();
        double ax = l_accel - r_accel;
        double ay = m_accel;

        double push = (ax == 0.0 && ay == 0.0) ? 0.0 : atan2(ax, ay) * 180.0 / PI;
        push = fmax(-MAX_TILT, fmin(MAX_TILT, push));
        robust_rotate(find_min_travel_angle(push - push_offset(t), angle));

        // Fire only with the part of the request the thruster can deliver in the
        // direction it points right now (zero if we're still turning the wrong way).
        double dir = (angle + push_offset(t)) * PI / 180.0;
        double along = ax * sin(dir) + ay * cos(dir);
        double power = normalize_value(fmax(0.0, along) / max_accel(t), Nthrust);

        Robust(Main_Thruster, t == MAIN_THRUSTER ? power : 0.0);
        Robust(Left_Thruster, t == LEFT_THRUSTER ? power : 0.0);
        Robust(Right_Thruster, t == RIGHT_THRUSTER ? power : 0.0);
    }

  public:
    // Powers are in [0, 1] and -1 leaves that thruster's request unchanged.
    void robust_thruster(double main_power, double left_power, double right_power) {
        if (main_power != -1)
            m_accel = fmin(fmax(main_power, 0.0), 1.0) * MT_ACCEL;
        if (left_power != -1)
            l_accel = fmin(fmax(left_power, 0.0), 1.0) * LT_ACCEL;
        if (right_power != -1)
            r_accel = fmin(fmax(right_power, 0.0), 1.0) * RT_ACCEL;
        apply();
    }

    void robust_rotate(double delta) {
        if (!isfinite(delta))
            return;
        delta = normalize_value(delta, Nangle);
        
        if (fabs(delta) < ROTATE_TOLERANCE)
            delta = 0.0;
        Robust(Rotate, delta / DELIVERY_RATIO);
    }
};

TaskControl *get_thruster() {
    static TaskControl s;
    return &s;
}

void Lander_Control(void) {
    /*
    This is the main control function for the lander. It attempts
    to bring the ship to the location of the landing platform
    keeping landing parameters within the acceptable limits.

    How it works:

    - First, if the lander is rotated away from zero-degree Robust(Angle),
      rotate lander back onto zero degrees.
    - Determine the horizontal distance between the lander and
      the platform, fire horizontal thrusters appropriately
      to change the horizontal velocity so as to decrease this
      distance
    - Determine the vertical distance to landing platform, and
      allow the lander to descend while keeping the vertical
      speed within acceptable bounds. Make sure that the lander
      will not hit the ground before it is over the platform!

    As noted above, this function assumes everything is working
    fine.
 */

    /*************************************************
   TO DO: Modify this function so that the ship safely
          reaches the platform even if components and
          sensors fail!

          Note that sensors are noisy, even when
          working properly.

          Finally, YOU SHOULD provide your own
          functions to provide sensor readings,
          these functions should work even when the
          sensors are faulty.

          For example: Write a function FVelocity_X_robust()
          which returns the module's horizontal velocity.
          It should determine whether the velocity
          sensor readings are accurate, and if not,
          use some alternate method to determine the
          horizontal velocity of the lander.

          NOTE: Your robust sensor functions can only
          use the available sensor functions and control
          functions!
          DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
          ACCESS THE SIMULATION STATE. That's cheating,
          I'll give you zero.
  **************************************************/

    double VXlim;
    double VYlim;

    TaskControl *tc = get_thruster();

    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.
    if (fabs(Robust(Position_X) - PLAT_X) > 200)
        VXlim = 25;
    else if (fabs(Robust(Position_X) - PLAT_X) > 100)
        VXlim = 15;
    else
        VXlim = 5;

    if (PLAT_Y - Robust(Position_Y) > 200)
        VYlim = -20;
    else if (PLAT_Y - Robust(Position_Y) > 100)
        VYlim = -10; // These are negative because they
    else
        VYlim = -4; // limit descent velocity

    // Ensure we will be OVER the platform when we land

    if (PLAT_Y - Robust(Position_Y) < ALIGN_ALTITUDE) {
        tc->robust_rotate(find_min_travel_angle(0.0, Robust(Angle)));

        double power = (Robust(Velocity_Y) < -1.0 && (fabs(Robust(Position_X) - PLAT_X) < PLAT_TOL )) ? 1.0 : 0.0;
        if (MT_OK)
            Robust(Main_Thruster,  normalize_value(power, Nthrust));
        else if (LT_OK)
            Robust(Left_Thruster,  normalize_value(power, Nthrust));
        else
            Robust(Right_Thruster, normalize_value(power, Nthrust));

        return; // skip everything else, no more floating
    }

    // IMPORTANT NOTE: The code below assumes all components working
    // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
    // fail. More likely, you will need a set of case-based code
    // chunks, each of which works under particular failure conditions.

    // Check for rotation away from zero degrees - Rotate first,
    // use thrusters only when not rotating to avoid adding
    // velocity components along the rotation directions
    // Note that only the latest Rotate() command has any
    // effect, i.e. the rotation Robust(Angle) does not accumulate
    // for successive calls.

    // Module is oriented properly, check for horizontal position
    // and set thrusters appropriately.
    if (Robust(Position_X) > PLAT_X) {
        // Lander is to the LEFT of the landing platform, use Right thrusters to move
        // lander to the left.
        tc->robust_thruster(-1, 0, -1);
        //Left_Thruster(0); // Make sure we're not fighting ourselves here!
        if (Robust(Velocity_X) > (-VXlim))
            tc->robust_thruster(-1, -1, (VXlim + fmin(0, Robust(Velocity_X))) / VXlim);
        //Right_Thruster((VXlim + fmin(0, Robust(Velocity_X))) / VXlim);
        else {
            // Exceeded velocity limit, brake
            //Right_Thruster(0);
            tc->robust_thruster(-1, fabs(VXlim - Robust(Velocity_X)), 0);
            //Left_Thruster(fabs(VXlim - Robust(Velocity_X)));
        }
    } else {
        // Lander is to the RIGHT of the landing platform, opposite from above
        // Right_Thruster(0);
        tc->robust_thruster(-1, -1, 0);
        if (Robust(Velocity_X) < VXlim)
            tc->robust_thruster(-1, (VXlim - fmax(0, Robust(Velocity_X))) / VXlim, -1);
        //Left_Thruster((VXlim - fmax(0, Robust(Velocity_X))) / VXlim);
        else {
            tc->robust_thruster(-1, 0, fabs(VXlim - Robust(Velocity_X)));
            //Left_Thruster(0);
            //Right_Thruster(fabs(VXlim - Robust(Velocity_X)));
        }
    }

    // Vertical adjustments. Basically, keep the module below the limit for
    // vertical velocity and allow for continuous descent. We trust
    // Safety_Override() to save us from crashing with the ground.
    if (Robust(Velocity_Y) < VYlim)
        tc->robust_thruster(1, -1, -1);
    //Main_Thruster(1.0);
    else
        //Main_Thruster(0);
        tc->robust_thruster(0, -1, -1);
}

void Safety_Override(void) {
    /*
    This function is intended to keep the lander from
    crashing. It checks the sonar distance array,
    if the distance to nearby solid surfaces and
    uses thrusters to maintain a safe distance from
    the ground unless the ground happens to be the
    landing platform.

    Additionally, it enforces a maximum speed limit
    which when breached triggers an emergency brake
    operation.
  */

    /**************************************************
   TO DO: Modify this function so that it can do its
          work even if components or sensors
          fail
  **************************************************/

    /**************************************************
    How this works:
    Check the sonar readings, for each sonar
    reading that is below a minimum safety threshold
    AND in the general direction of motion AND
    not corresponding to the landing platform,
    carry out speed corrections using the thrusters
  **************************************************/

    double DistLimit;
    double Vmag;
    double dmin;

    TaskControl *tc = get_thruster();

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag = Robust(Velocity_X) * Robust(Velocity_X);
    Vmag += Robust(Velocity_Y) * Robust(Velocity_Y);

    DistLimit = fmax(75, Vmag);

    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
    if (fabs(PLAT_X - Robust(Position_X)) < 150 && fabs(PLAT_Y - Robust(Position_Y)) < 150)
        return;

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.
    dmin = 1000000;
    if (Robust(Velocity_X) > 0) {
        for (int i = 5; i < 14; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
    } else {
        for (int i = 22; i < 32; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
    }
    // Determine whether we're too close for comfort. There is a reason
    // to have this distance limit modulated by horizontal speed...
    // what is it?
    if (dmin < DistLimit * fmax(.25, fmin(fabs(Robust(Velocity_X)) / 5.0, 1))) { // Too close to a surface in the horizontal direction

        if (Robust(Velocity_X) > 0) {
            tc->robust_thruster(-1, 0, 1);
            //Right_Thruster(1.0);
            //Left_Thruster(0.0);
        } else {
            tc->robust_thruster(-1, 1, 0);
            //Left_Thruster(1.0);
            //Right_Thruster(0.0);
        }
    }

    // Vertical direction
    dmin = 1000000;
    if (Robust(Velocity_Y) > 5) // Mind this! there is a reason for it...
    {
        for (int i = 0; i < 5; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
        for (int i = 32; i < 36; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
    } else {
        for (int i = 14; i < 22; i++)
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
    }
    if (dmin < DistLimit) // Too close to a surface in the horizontal direction
    {
        if (Robust(Velocity_Y) > 2.0) {
            //Main_Thruster(0.0);
            tc->robust_thruster(0, -1, -1);
        } else {
            //Main_Thruster(1.0);
            tc->robust_thruster(1, -1, -1);
        }
    }
}
