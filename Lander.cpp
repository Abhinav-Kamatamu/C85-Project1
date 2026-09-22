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
          * Maximum landing angle should be less than 15 degrees w.r.t vertical

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

          Velocity_X();  - Gives you the lander's horizontal velocity
          Velocity_Y();	 - Gives you the lander's vertical velocity
          Position_X();  - Gives you the lander's horizontal position (0 to 1024)
          Position_Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

          SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
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

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
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
          Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
                                           (ccw if angle is negative) from current
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
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

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
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "Lander_Control.h"

#define FLOATING_TOLERANCE 0.00001f

int a = 1;
void robust_rotate(double angle) {
	Rotate(angle);
}

int safe_posx(double *posx){
	// We want to return -1 in case of failure.
	// Ideally, we use some sort of something here to check if Position_X is working correctly over a few samples.
	*posx = Position_X();
	return 1;
}

struct game_state {
	// int initial = 1;
	double pos[2];
	double vel[2];
	double accel[2];
	double angle;
	double sonar[36];
    double time = 0;
};

void get_initial_game_state(struct game_state *state) {
	/**
		TODO: MAKE SURE THESE USE THE SAFE EQUIVALENTS 
	*/
	safe_posx(&state->pos[0]);
	state->pos[1] = Position_Y();
	state->vel[0] = Velocity_X();
	state->vel[1] = Velocity_Y();
	state->accel[0] = 0;
	state->accel[1] = G_ACCEL;
	state->angle = Angle();
	for (int i = 0; i < 36; i++) {
		state->sonar[i] = SONAR_DIST[i];
	}
}

void solve_equation_1d(double *u, double *v, double *a, double *t, double *s, char which) {
	// Solves any of the following equations:
	// v = u + at
	// v^2 = u^2 + 2as
	// s = ut + 1/2 at^2

	switch (which){
		case 'v':
			if (t == nullptr) {
	            *v = fsqrt((*u) * (*u) + 2 * (*a) * (*s));
			} else {
				*v = *u + *a * *t;
			}
			break;
		case 'u':
			if (t == nullptr) {
				*u = fsqrt((*v) * (*v) - 2 * (*a) * (*s));
			} else {
				*u = *v - *a * *t;
			}
			break;
		case 'a':
			if (t == nullptr) {
				*a = ((*v) * (*v) - (*u) * (*u)) / (2 * (*s));
			} else {
				*a = (*v - *u) / *t;
			}
			break;
		case 't':
			if (a == nullptr) {
				*t = ((*v) - (*u)) / (*a);
			} else {
				*t = ((*v) * (*v) - (*u) * (*u)) / (2 * (*a) * (*s));
			}
			break;
		case 's':
			if (t == nullptr) {
				*s = ((*v) * (*v) - (*u) * (*u)) / (2 * (*a));
			} else {
				*s = (*u) * (*t) + 0.5 * (*a) * (*t) * (*t);
			}
			break;
		default:
			std::cerr << "Invalid equation type" << std::endl;
			break;
	}
}




void solve_equation_2d(double u[2], double v[2], double a[2], double *t, double s[2], char which) {
    switch (which) {
        case 'v':
            solve_equation_1d(&u[0], &v[0], &a[0], t, &s[0], 'v');
            solve_equation_1d(&u[1], &v[1], &a[1], t, &s[1], 'v');
            break;
        case 'u':
            solve_equation_1d(&u[0], &v[0], &a[0], t, &s[0], 'u');
            solve_equation_1d(&u[1], &v[1], &a[1], t, &s[1], 'u');
            break;
        case 'a':
            solve_equation_1d(&u[0], &v[0], &a[0], t, &s[0], 'a');
            solve_equation_1d(&u[1], &v[1], &a[1], t, &s[1], 'a');
            break;
        case 't': {
            // Solve each axis independently, then reconcile.
            double t0 = 0.0, t1 = 0.0;
            solve_equation_1d(&u[0], &v[0], &a[0], &t0, &s[0], 't');
            solve_equation_1d(&u[1], &v[1], &a[1], &t1, &s[1], 't');
            if (fabs(t0 - t1) > FLOATING_TOLERANCE) {
                std::cerr << "Warning: x/y solutions for t disagree ("
                          << t0 << " vs " << t1 << "). Using the average."
                          << std::endl;
            }
            *t = (t0 + t1) / 2.0;
            break;
        }
        case 's':
            solve_equation_1d(&u[0], &v[0], &a[0], t, &s[0], 's');
            solve_equation_1d(&u[1], &v[1], &a[1], t, &s[1], 's');
            break;
        default:
            std::cerr << "Invalid equation type" << std::endl;
            break;
    }
}


struct Action{
    enum action_type {THRUST, ROTATE} type;
    double value;    // Rotations in degress 
                     // Or thrust in accelerations
    double time;     // Duration of Action
                     // Rotations will not need any time argument (Can be 0)
                     // Thurst actions will require you to specify how long to thrust for.
    int in_progress; // A bool to see if the action is running. (0 = not started, 1 = in progress, 2 = completed)
    int start_time;
};
class GameControler {
    game_state state;

    
public:
    class ActionHandler{
        std::vector<Action> act_bck;
        public:
        
        double is_rotating = 0; // If non-zero, shows how many degrees still to be rotated.
        ActionHandler(){
            act_bck = std::vector<Action>();
        }

        void optimise_actions(){
            /** TODO: Implement a function to handle this */
            return;
        }

        void add_action(Action act){
            act_bck.push_back(act);
            optimise_actions();
        }

        void clean_completed_actions(){
            for(int i = 0; i < act_bck.size(); i++){
                if (act_bck[i].in_progress == 2){
                    std::swap(act_bck[i], act_bck.back());
                    act_bck.pop_back();
                    i--;
                }
            }
        }

        void run_actions(game_state &state){
            for (Action &act : act_bck){
                if (act.in_progress == 0){
                    act.in_progress = 1;
                    switch (act.type){
                        case Action::THRUST:
                            if (is_rotating == 0){
                                Main_Thruster(act.value);
                                act.start_time = state.time;
                                std::cerr << "Starting thrust\n";
                            }
                            else{
                                act.in_progress = 0;
                            }
                            break;
                        case Action::ROTATE:
                            robust_rotate(act.value);
                            is_rotating = act.value;
                            std::cerr << "Starting rotation\n";
                            break;
                    }
                } else {
                    if (act.type == Action::THRUST){
                        if (state.time >= act.start_time + act.time){
                            Main_Thruster(0.0);
                            act.in_progress = 2; // Mark as completed
                        }
                    } else {
                        act.in_progress = 2; // Mark as completed
                    }
                }
            }

            clean_completed_actions();
        }

    };

    ActionHandler action_handler;

    GameControler(){
        get_initial_game_state(&state);
        action_handler = ActionHandler();
        stabilise(0.5);
    }

    void get_initial_state(){
        safe_posx(&(state.pos[0]));
        state.pos[1] = Position_Y();
        state.vel[0] = Velocity_X();
        state.vel[1] = Velocity_Y();
        state.accel[0] = 0;
        state.accel[1] = -G_ACCEL;
        state.angle = Angle();
        for (int i = 0; i < 36; i++) {
            state.sonar[i] = SONAR_DIST[i];
        }
    }

    void stabilise(double target_time) {
        double u[2] = {state.vel[0], state.vel[1]}; // current velocity
        double v[2] = {0.0, 0.0};                   // we want zero
        double required_accel[2] = {0.0, 0.0};
        double s[2] = {0.0, 0.0}; // unused by 'a'.

        solve_equation_2d(u, v, required_accel, &target_time, s, 'a');

        required_accel[1] += G_ACCEL; // thruster must also cancel gravity and kill vel.

        double required_angle = atan2(required_accel[0], required_accel[1]) * 180.0 / PI;
        double thrust_mag = sqrt(required_accel[0] * required_accel[0] +
                                required_accel[1] * required_accel[1]);

        std::cerr << "Thrust magnitude: " << thrust_mag << " Required angle: " << required_angle << "\n";

        action_handler.add_action({Action::ROTATE, required_angle - state.angle, 0.0, 0});
        // robust_rotate(required_angle - state.angle);
        action_handler.add_action({Action::THRUST, thrust_mag, target_time, 0});
    }

    void tick(){
        std::cerr << action_handler.is_rotating << "\r";
        action_handler.run_actions(state);
        if(fabs(action_handler.is_rotating) >= 500){
            action_handler.is_rotating -= state.time * MAX_ROT_RATE * 180.0 / PI;
        } else {
            action_handler.is_rotating = 0;
        }
        state.time += T_STEP;

        if (action_handler.act_bck.empty()){
            action_handler.add_action()
        }
    }

};

/*************************************************************************************************/
/*************************** The actual code is below this line **********************************/

void Lander_Control(void) {

}

void Safety_Override(void) {
    static GameControler gc;   // constructed on first call, not at static-init time
    gc.tick();
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
   TODO: Modify this function so that it can do its
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

//   double DistLimit;
//   double Vmag;
//   double dmin;

//   // Establish distance threshold based on lander
//   // speed (we need more time to rectify direction
//   // at high speed)
//   Vmag = Velocity_X() * Velocity_X();
//   Vmag += Velocity_Y() * Velocity_Y();

//   DistLimit = fmax(75, Vmag);

//   // If we're close to the landing platform, disable
//   // safety override (close to the landing platform
//   // the Control_Policy() should be trusted to
//   // safely land the craft)
//   if (fabs(PLAT_X - Position_X()) < 150 && fabs(PLAT_Y - Position_Y()) < 150)
//     return;

//   // Determine the closest surfaces in the direction
//   // of motion. This is done by checking the sonar
//   // array in the quadrant corresponding to the
//   // ship's motion direction to find the entry
//   // with the smallest registered distance

//   // Horizontal direction.
//   dmin = 1000000;
//   if (Velocity_X() > 0) {
//     for (int i = 5; i < 14; i++)
//       if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
//         dmin = SONAR_DIST[i];
//   } else {
//     for (int i = 22; i < 32; i++)
//       if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
//         dmin = SONAR_DIST[i];
//   }
//   // Determine whether we're too close for comfort. There is a reason
//   // to have this distance limit modulated by horizontal speed...
//   // what is it?
//   if (dmin < DistLimit * fmax(.25, fmin(fabs(Velocity_X()) / 5.0, 1))) { // Too close to a surface in the horizontal direction
//     if (Angle() > 1 && Angle() < 359) {
//       if (Angle() >= 180)
//         Rotate(360 - Angle());
//       else
//         Rotate(-Angle());
//       return;
//     }

//     if (Velocity_X() > 0) {
//       Right_Thruster(1.0);
//       Left_Thruster(0.0);
//     } else {
//       Left_Thruster(1.0);
//       Right_Thruster(0.0);
//     }
//   }

//   // Vertical direction
//   dmin = 1000000;
//   if (Velocity_Y() > 5) // Mind this! there is a reason for it...
//   {
//     for (int i = 0; i < 5; i++)
//       if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
//         dmin = SONAR_DIST[i];
//     for (int i = 32; i < 36; i++)
//       if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
//         dmin = SONAR_DIST[i];
//   } else {
//     for (int i = 14; i < 22; i++)
//       if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
//         dmin = SONAR_DIST[i];
//   }
//   if (dmin < DistLimit) // Too close to a surface in the horizontal direction
//   {
//     if (Angle() > 1 || Angle() > 359) {
//       if (Angle() >= 180)
//         Rotate(360 - Angle());
//       else
//         Rotate(-Angle());
//       return;
//     }
//     if (Velocity_Y() > 2.0) {
//       Main_Thruster(0.0);
//     } else {
//       Main_Thruster(1.0);
//     }
//   }
}
