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
#include <string.h>
#include <vector>
#include <string.h>
#include "Lander_Control.h"

#define UP_ACCEL 25

#define FLOATING_TOLERANCE 0.00001f
#define HOVER_HEIGHT 70
#define NSAMPLES 60
#define HOVER_HEIGHT 70
#define NSAMPLES 60

int a = 1;

struct game_state {
	// int initial = 1;
	// int initial = 1;
	double pos[2];
	double vel[2];
	double accel[2];
	double angle;
	double sonar[36];
    double time = 0;
};

double normalize_angle(double a) {
	while (a > 180.0)   a -= 360.0;
	while (a <= -180.0) a += 360.0;
	return a;
}

#define ROTATE_DELIVERY_RATIO 0.9507   // measured empirically
// Rotate() consistently delivers ~95.07% of whatever asked
// took into account Angle sensor noise and rotation time

#define ROTATE_TOLERANCE 3.0   // degrees - skip re-rotating for corrections this small
                                 // (without this, any nonzero residual misalignment
                                 // re-triggers a full rotate cycle and thrust never sustains)

// We assume single-threaded operation.
static double rotate_start_time = 0.0;
static double rotate_duration   = 0.0;


void robust_rotate(double delta, struct game_state &state) {
    if (!isfinite(delta)) return;

    delta = normalize_angle(delta);

    state.angle = normalize_angle(state.angle + delta);
    // calculated estimate of where we'll end up, the theoretical delta

    double compensated = delta / ROTATE_DELIVERY_RATIO;
    // overshoot, since we know it delivers ~95% of it

    double compensated_rad = fabs(compensated) * PI / 180.0;
    rotate_duration   = (compensated_rad / MAX_ROT_RATE) * T_STEP;
    rotate_start_time = state.time;

    Rotate(compensated);
}

// Returns 1 once enough simulated time has passed for the most recent
// robust_rotate call to physically finish, 0 while still waiting.
// Just timing, no sensor involved
int robust_rotate_status(struct game_state &state) {
    return (state.time - rotate_start_time >= rotate_duration) ? 1 : 0;
}

double Angle_Robust() {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++)
        sum += Angle();
    return sum / NSAMPLES;
}

double Position_X_Robust() {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++)
        sum += Position_X();
    return sum / NSAMPLES;
}

double Position_Y_Robust() {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++)
        sum += Position_Y();
    return sum / NSAMPLES;
}

double Velocity_X_Robust() {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++)
        sum += Velocity_X();
    return sum / NSAMPLES;
}

double Velocity_Y_Robust() {
    double sum = 0;
    for (int i = 0; i < NSAMPLES; i++)
        sum += Velocity_Y();
    return sum / NSAMPLES;
}



void print_state(game_state &state){
    static FILE *fp = fopen("lander_state.txt", "w");
        fprintf(fp, "\r\033[%dA\r\033[2K", 12);
        fprintf(fp, "Position: (%.2f, %.2f)\n", state.pos[0], state.pos[1]);
        fprintf(fp, "Real Position: (%.2f, %.2f)\n", Position_X_Robust(), Position_Y_Robust());
        fprintf(fp, "Position off by (%): %.2f\n\n\n\n", fsqrt((state.pos[0] - Position_X_Robust()) * (state.pos[0] - Position_X_Robust()) + (state.pos[1] - Position_Y_Robust()) * (state.pos[1] - Position_Y_Robust())) / fsqrt(state.pos[0] * state.pos[0] + state.pos[1] * state.pos[1]) * 100);
        fprintf(fp, "Velocity: (%.2f, %.2f)\n", state.vel[0], state.vel[1]);
        fprintf(fp, "Real Velocity: (%.2f, %.2f)\n", Velocity_X_Robust(), Velocity_Y_Robust());
        fprintf(fp, "Acceleration: (%.2f, %.2f)\n", state.accel[0], state.accel[1]);
        fprintf(fp, "Angle: %.2f\n", state.angle);
        fprintf(fp, "Real Angle: %.2f\n", (Angle_Robust() > 180 ? Angle_Robust() - 360 : Angle_Robust()));
        fprintf(fp, "Time: %.2f\n", state.time);
        fflush(fp);
    }

void solve_equation_1d(double *u, double *v, double *a, double *t, double *s, char which) {
	// Solves any of the following equations:
	// v = u + at
	// v^2 = u^2 + 2as
	// s = ut + 1/2 at^2

	switch (which){
		case 'v':
			if (t == nullptr) {
	            *v = copysign(fsqrt((*u) * (*u) + 2 * (*a) * (*s)), *s);
			} else {
				*v = *u + *a * *t;
			}
			break;
		case 'u':
			if (t == nullptr) {
				*u = copysign(fsqrt((*v) * (*v) - 2 * (*a) * (*s)), *s);
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
			if (s == nullptr) {
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
    enum action_type {THRUST, ROTATE, IDLE} type;
    double value;    // Rotations in degress
                     // Or thrust in accelerations
    double duration;     // Duration of Action
                     // Thurst actions will require you to specify how long to thrust for.
    int in_progress; // A bool to see if the action is running. (0 = not started, 1 = in progress, 2 = completed)
    double start_time;

    int is_parallel; // A bool to see if the action can be run in parallel with prev actions. (0 = no, 1 = yes)

    // game_state future_state; // A representation of the what the future state "should" look like.

    // robust_thruster's own progress - lives here, per-Action, instead of
    // static locals, so multiple queued thrust actions each track their
    // own state independently instead of sharing one hidden global copy.
    int    chosen_thruster = -1;   // 0 = Main, 1 = Left, 2 = Right
    double chosen_power    = 0.0;
    int    rt_phase        = 0;    // 0 = deciding/rotating, 1 = thrusting
};


void robust_thruster(Action &act, struct game_state &state) {
    double accel_magnitude = act.value;

    // If we already picked a thruster and it's died since (e.g. mid-burn), force a re-pick
    if (act.chosen_thruster != -1) {
        int still_ok = (act.chosen_thruster == 0) ? MT_OK
                      : (act.chosen_thruster == 1) ? LT_OK : RT_OK;
        if (!still_ok) {
            act.rt_phase = 0;
        }
    }

    if (act.rt_phase == 0) {
        double target_push_angle = state.angle;
        const double MAIN_OFFSET = 0.0, LEFT_OFFSET = 90.0, RIGHT_OFFSET = -90.0;

        int best = -1;
        double best_delta = 0.0, best_abs = 1e9;
        if (MT_OK) { double d = normalize_angle(target_push_angle - (state.angle + MAIN_OFFSET));
                     if (fabs(d) < best_abs) { best_abs = fabs(d); best_delta = d; best = 0; } }
        if (LT_OK) { double d = normalize_angle(target_push_angle - (state.angle + LEFT_OFFSET));
                     if (fabs(d) < best_abs) { best_abs = fabs(d); best_delta = d; best = 1; } }
        if (RT_OK) { double d = normalize_angle(target_push_angle - (state.angle + RIGHT_OFFSET));
                     if (fabs(d) < best_abs) { best_abs = fabs(d); best_delta = d; best = 2; } }

        double accel_const = (best == 0) ? MT_ACCEL : (best == 1) ? LT_ACCEL : RT_ACCEL;
        act.chosen_power    = fmin(accel_magnitude / accel_const, 1.0);
        act.chosen_thruster = best;

        if (fabs(best_delta) > ROTATE_TOLERANCE) {
            Main_Thruster(0.0); Left_Thruster(0.0); Right_Thruster(0.0);  // no thrust while turning
            robust_rotate(best_delta, state);
            act.duration += rotate_duration;   // extend the time budget to cover the redirect too
            act.rt_phase = 2;
            return;
        }

        act.rt_phase = 1;

    } else if (act.rt_phase == 2) {
        if (!robust_rotate_status(state)) return;   // still turning
        act.rt_phase = 1;
    }

    // Actually thrusting
    // project along the direction the ACTIVE thruster pushes
    const double THRUSTER_OFFSET[3] = {0.0, 90.0, -90.0};
    double push_angle = state.angle + THRUSTER_OFFSET[act.chosen_thruster];
    state.accel[0] = act.value * sin(push_angle * PI / 180.0);
    state.accel[1] = act.value * cos(push_angle * PI / 180.0) - G_ACCEL;

    Main_Thruster (act.chosen_thruster == 0 ? act.chosen_power : 0.0);
    Left_Thruster (act.chosen_thruster == 1 ? act.chosen_power : 0.0);
    Right_Thruster(act.chosen_thruster == 2 ? act.chosen_power : 0.0);
}

class GameControler {
    game_state state;
    game_state future_state;

    
public:
    class ActionHandler{
        public:
        std::vector<Action> act_bck;
        int is_running = 0;
        
        ActionHandler(){
            act_bck = std::vector<Action>();
        }

        void optimise_actions(){
            /** TODO: Implement a function to handle this */
            return;
        }

        void add_action(Action act){
            act.in_progress = 0;
            act_bck.push_back(act);
            optimise_actions();
        }

        void start_action(Action &act, game_state &state){
            act.in_progress = 1;
            act.start_time = state.time;
            switch (act.type){
                case Action::THRUST:
                    std::cerr << "Thrusting with " << act.value << "\n";

                    robust_thruster(act, state);

                    state.accel[0] = act.value * sin(state.angle * PI / 180.0);
                    state.accel[1] = act.value * cos(state.angle * PI / 180.0) - G_ACCEL;
                    
                    break;
                case Action::ROTATE:
                    std::cerr << "Rotating with " << act.value << "\n";

                    state.accel[0] = 0.0;
                    state.accel[1] = -G_ACCEL;

                    robust_rotate(act.value, state);
                    act.duration = rotate_duration; // Update the duration of the action to match the rotation time
                    break;
                case Action::IDLE:
                    std::cerr << "Idling for " << act.duration << "\n";
                    break;
            }
            process_state_updates(state);
        }

        void continue_action(Action &act, game_state &state){

            // If the action is finished, then we delete it
            if (state.time - act.start_time >= act.duration){
                act.in_progress = 2;
                switch (act.type){
                    case Action::THRUST:
                        std::cerr << "Stopping thrust\n";
                        robust_thruster(act, state);

                        std::cerr << "\n\n\n===================================\n";
                        std::cerr << "After Thrusting\n";
                        std::cerr << "State Position (" << state.pos[0] << ", " << state.pos[1] << ")\n";
                        std::cerr << "State Velocity (" << state.vel[0] << ", " << state.vel[1] << ")\n";
                        std::cerr << "Actual Position (" << Position_X_Robust() << ", " << Position_Y_Robust() << ")\n";
                        std::cerr << "Actual Velocity (" << Velocity_X_Robust() << ", " << Velocity_Y_Robust() << ")\n";
                        std::cerr << "=====================================\n\n\n";
                        break;
                    case Action::ROTATE:
                        std::cerr << "Rotation completed of " << act.value << "\n";
                        std::cerr << "Current angle: " << Angle_Robust() << "\n";

                        std::cerr << "\n\n\n===================================\n";
                        std::cerr << "After rotation\n";
                        std::cerr << "State Position (" << state.pos[0] << ", " << state.pos[1] << ")\n";
                        std::cerr << "State Velocity (" << state.vel[0] << ", " << state.vel[1] << ")\n";
                        std::cerr << "Actual Position (" << Position_X_Robust() << ", " << Position_Y_Robust() << ")\n";
                        std::cerr << "Actual Velocity (" << Velocity_X_Robust() << ", " << Velocity_Y_Robust() << ")\n";
                        std::cerr << "=====================================\n\n\n";

                        // Do nothing, rotation is a single step command
                        break;
                    case Action::IDLE:
                        std::cerr << "Idle completed\n";
                        std::cerr << "Current velocity: (" << Velocity_X_Robust() << ", " << Velocity_Y_Robust() << ")\n";
                        std::cerr << "Current state velocity: (" << state.vel[0] << ", " << state.vel[1] << ")\n";
                        break;
                }
            }

            // Else, we continue the action: updating the game state variables
            else{
                switch (act.type){
                    case Action::THRUST:
                        // Update the game state variables
                        state.accel[0] = act.value * sin(state.angle * PI / 180.0);
                        state.accel[1] = act.value * cos(state.angle * PI / 180.0) - G_ACCEL;
                        break;
                    case Action::ROTATE:
                        // Angles will be stored between -180 and 180. 
                        // We will add hte change in rotation expected and see if it is in this range.
                        // If not, we will adjust accordingly.
                        {
                            double angle_change = act.value * (T_STEP / act.duration);
                            // state.angle += angle_change;
                            // Ensure the angle is within the range [-180, 180]
                            if (state.angle > 180.0) {
                                state.angle -= 360.0;
                            } else if (state.angle < -180.0) {
                                state.angle += 360.0;
                            }
                        }
                        break;
                    case Action::IDLE:
                        // Update the game state variables with only gravity
                        state.accel[0] = 0.0;
                        state.accel[1] = -G_ACCEL;
                        break;
                }
            }
            print_state(state);
            // std::cerr << "XHIST: " << Xhist << "\r";
        }

        void clean_completed_actions(){
            for(int i = 0; i < act_bck.size(); i++){
                if (act_bck[i].in_progress == 2){
                    act_bck.erase(act_bck.begin() + i);
                    i--;
                }
            }
        }

        void process_state_updates(game_state &state){
            // Update the game state variables with only gravity
            state.vel[0] += state.accel[0] * T_STEP;
            state.vel[1] += state.accel[1] * T_STEP;
            state.pos[0] += state.vel[0] * T_STEP;
            state.pos[1] -= state.vel[1] * T_STEP;  // Subtract to account for flipped coordinates
        }

        void run_actions(game_state &state){
            int detected_all_completed = 1;

            // Reset acceleration to gravity only before processing actions
            state.accel[0] = 0.0;
            state.accel[1] = -G_ACCEL;

            for (Action &act : act_bck){
                switch (is_running){
                    case 0:
                        start_action(act, state);
                        detected_all_completed = 0;
                        is_running = 1;
                        break;
                    
                    case 1:
                        if (act.is_parallel && act.in_progress == 0){
                            start_action(act, state);
                            detected_all_completed = 0;
                        }
                        if (act.in_progress == 1){
                            continue_action(act, state);
                            detected_all_completed = 0;
                        }
                        break;
                }
            }

            // process_state_updates(state);

            if (detected_all_completed) {
                is_running = 0;
            }
            clean_completed_actions();
        }

    };

    ActionHandler action_handler;
    enum Phase {STABILISE, GO_UP, GO_HORIZONTAL, GO_DOWN} phase = STABILISE;

    GameControler(){
        get_initial_state();
        action_handler = ActionHandler();
        stabilise(2);
    }

    void get_initial_state(){
        state.pos[0] = Position_X_Robust();
        state.pos[1] = Position_Y_Robust();
        std::cerr << "Initial Position: (" << state.pos[0] << ", " << state.pos[1] << ")\n";
        state.vel[0] = Velocity_X_Robust();
        state.vel[1] = Velocity_Y_Robust();
        state.accel[0] = 0;
        state.accel[1] = -G_ACCEL;
        state.angle = Angle_Robust();
        for (int i = 0; i < 36; i++) {
            state.sonar[i] = SONAR_DIST[i];
        }
    }

    void next_phase(){
        switch(phase){
            case STABILISE:
                phase = GO_UP;
                go_up(future_state);
                break;
            case GO_UP:
                phase = GO_HORIZONTAL;
                // go_horizontal(state);
                break;
            case GO_HORIZONTAL:
                phase = GO_DOWN;
                // go_down(state);
                break;
            case GO_DOWN: // I don't think we will ever reach this phase...
                phase = GO_DOWN;
                break;
        }
    }
    

    double find_min_travel_angle(double required_angle, double state_angle){
            double diff = required_angle - state_angle;
            diff = fmod(diff, 360.0);
            if(diff <= -180.0){
                diff += 360.0;
            } else if (diff > 180.0){
                diff -= 360.0;
            }

            return diff;
    }

    void stabilise(double target_time) {
        get_initial_state();
        future_state = state;

        double u[2] = {state.vel[0], state.vel[1]}; // current velocity
        double v[2] = {0.0, 0.0};                   // we want zero
        double required_accel[2] = {0.0, 0.0};
        double time_offset = 0.0;   // The amount of time it will take to rotate to the required angle
                                    // Don't start the thruster until this is completed
        double s[2] = {0.0, 0.0};   // unused by 'a'.
        double G_accel[2] = {0.0, -G_ACCEL};

        double thrust_mag;
        double required_angle_time;
        double destination_angle;
        double min_rot_angle;

        for(int i = 0; i < 10; i++){
            // Itterate simulation steps for time-step accuracy in calculations:
            double u_after[2];
            memset(required_accel, 0, sizeof(required_accel));
            
            solve_equation_2d(u, u_after, G_accel, &time_offset, s, 'v'); // Account for effect of gravity on u while rotating
            solve_equation_2d(u_after, v, required_accel, &target_time, s, 'a'); // Calculate the required thrust acceleration
            solve_equation_2d(u_after, v, required_accel, &target_time, s, 's'); // Calculate the distance travelled

            required_accel[1] += G_ACCEL; // thruster must also cancel gravity and kill vel.

            destination_angle = atan2(required_accel[0], required_accel[1]) * 180.0 / PI;
            
            min_rot_angle = find_min_travel_angle(destination_angle, state.angle);
            
            // robust_rotate rotates by angle / ROTATE_DELIVERY_RATIO, so it takes longer...
            required_angle_time = (fabs(min_rot_angle) / ROTATE_DELIVERY_RATIO / (MAX_ROT_RATE * 180.0 / PI)) * T_STEP;
            time_offset = required_angle_time;
        }
        
        thrust_mag = sqrt(required_accel[0] * required_accel[0] +
                                required_accel[1] * required_accel[1]);

        // time for the second rotation
        double required_angle_time_2 = (fabs(destination_angle) / ROTATE_DELIVERY_RATIO / (MAX_ROT_RATE * 180.0 / PI)) * T_STEP;
        double hover_time = 1.0;

        std::cerr << "Thrust magnitude: " << thrust_mag << ";\n Change Angle: " << min_rot_angle << "\n";
                                //    type  ---------- value ----------- duration ------- is_parallel
        action_handler.add_action({Action::ROTATE, min_rot_angle, required_angle_time, .is_parallel=0});
        action_handler.add_action({Action::THRUST, thrust_mag, target_time, .is_parallel=0});

        std::cerr << "rotating: " << -destination_angle << "\n";
        action_handler.add_action({Action::ROTATE, -destination_angle, required_angle_time_2, .is_parallel=0});
        action_handler.add_action({Action::THRUST, G_ACCEL, hover_time, .is_parallel=0});

        // displacement during the first rotation
        double u_rot1_end[2];
        double s_rot1[2];
        solve_equation_2d(u, u_rot1_end, G_accel, &required_angle_time, s_rot1, 's');

        // second rotation, starting from rest, free fall
        double v_rot2_end[2];
        double s_rot2[2];
        solve_equation_2d(v, v_rot2_end, G_accel, &required_angle_time_2, s_rot2, 'v');
        solve_equation_2d(v, v_rot2_end, G_accel, &required_angle_time_2, s_rot2, 's');

        // velocity from rotation 2 is carried through
        double zero_accel[2] = {0.0, 0.0};
        double v_hover_end[2];
        double s_hover[2];
        solve_equation_2d(v_rot2_end, v_hover_end, zero_accel, &hover_time, s_hover, 'v');
        solve_equation_2d(v_rot2_end, v_hover_end, zero_accel, &hover_time, s_hover, 's');

        future_state.vel[0] = v_hover_end[0];
        future_state.vel[1] = v_hover_end[1];
        future_state.accel[0] = 0;
        future_state.accel[1] = -G_ACCEL;
        future_state.angle = 0;
        future_state.pos[0] += s_rot1[0] + s[0] + s_rot2[0] + s_hover[0];
        future_state.pos[1] -= s_rot1[1] + s[1] + s_rot2[1] + s_hover[1];
        future_state.time += required_angle_time + target_time + required_angle_time_2 + hover_time;

        std::cerr << "\n\n\n===================================\n";
        std::cerr << "Initial Position (" << state.pos[0] << ", " << state.pos[1] << ")\n";
        std::cerr << "Position after to rotation: (" << state.pos[0] + s_rot1[0] << ", " << state.pos[1] + s_rot1[1] << ")\n";
        std::cerr << "Position after to thrust: (" << state.pos[0] + s[0] << ", " << state.pos[1] + s[1] << ")\n";
        std::cerr << "Position after to second rotation: (" << state.pos[0] + s_rot2[0] << ", " << state.pos[1] + s_rot2[1] << ")\n";
        std::cerr << "Position after to hover: (" << state.pos[0] + s_hover[0] << ", " << state.pos[1] + s_hover[1] << ")\n";
        std::cerr << "Final Position (" << future_state.pos[0] << ", " << future_state.pos[1] << ")\n";
        std::cerr << "Final Velocity (" << future_state.vel[0] << ", " << future_state.vel[1] << ")\n";
        std::cerr << "Final Acceleration (" << future_state.accel[0] << ", " << future_state.accel[1] << ")\n";
        std::cerr << "Final Angle (" << future_state.angle << ")\n";
        std::cerr << "=====================================\n\n\n";
    }

    void go_up(game_state state) {
        /**
         * Let h be the vertical distance from present postion to the hover height.
         * We will accelerate for the h1 meters and then coast for the remaining h2 meters.
         * Acceleration will take t1 time
         * Coasting will take t2 time
         * 
         * Let u be the initial velocity
         * Let v be the final velocity after the acceleration phase
         * Let A be the vertical acceleration
         * 
         * This means, we solve for the variables t1 and t2 using the following equations:
         * h1 = u * t1 + 0.5 * (A - G_ACCEL) * t1^2
         * h2 = v * t2 + 0.5 * (-G_ACCEL) * t2^2
         * 2gh2 = v^2
         * v = u + (A - G_ACCEL) * t1
         * h1 + h2 = h
         * 
         * With this we get required v = \sqrt{\frac{ g(u^2 + 2(a - G_ACCEL) h) }{a} }
         * t1 = \frac{v - u}{a - G_ACCEL}
         * t2 = \frac{v}{G_ACCEL}
         * 
         */

        const double hover_height = HOVER_HEIGHT;
        const double A = UP_ACCEL;   // How fast do we want to go up?
        const double G = G_ACCEL;

        double u = state.vel[1]; // u_y (up-positive)
        double h = state.pos[1] - hover_height; // Height to move (up-positive)

        double v; // velocity after acceleration phase
        double t1 = 0.0;  // duration of acceleration phase
        double t2 = 0.0;  // duration of coasting phase
        bool   up_first;  // true: thrust then idle. false: idle then thrust.

        if (h >= 0.0) {
            // Need to go UP.
            v  = sqrt((G * (u * u + 2 * (A - G) * h)) / A);
            t1 = (v - u) / (A - G);
            t2 = v / G;
            up_first = true;
        } else {
            // Need to go DOWN.
            double dist_down = -h;
            double u_down    = -u;
            double v_down    = sqrt(((A - G) * (u_down * u_down + 2 * G * dist_down)) / A);
            t1 = (v_down - u_down) / G;
            t2 = v_down / (A - G);
            up_first = false;
        }

        // Just to be safe. Never can be sure about quadratics ;(
        t1 = fmax(t1, 0.0);
        t2 = fmax(t2, 0.0);

        std::cerr << "Going to hover height: " << hover_height << " from current height: " << state.pos[1] << "\n";
        std::cerr << "Currently we are at Position: (" << Position_X_Robust() << ", " << Position_Y_Robust() << ")\n\n\n";
        std::cerr << "Direction: " << (h >= 0.0 ? "UP" : "DOWN") << "\n";
        std::cerr << "Phase 1 (" << (up_first ? "thrust" : "idle") << "), duration: " << t1 << "\n";
        std::cerr << "Phase 2 (" << (up_first ? "idle" : "thrust") << "), duration: " << t2 << "\n";

        if (up_first) {
            action_handler.add_action({Action::THRUST, A, t1, .is_parallel=0});
            action_handler.add_action({Action::IDLE, 0.0, t2, .is_parallel=0});
        } else {
            action_handler.add_action({Action::IDLE, 0.0, t1, .is_parallel=0});
            action_handler.add_action({Action::THRUST, A, t2, .is_parallel=0});
        }
    }

    void tick(){
        if (action_handler.act_bck.empty()) {
            next_phase();
        }
        action_handler.run_actions(state);
        state.time += T_STEP;
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
