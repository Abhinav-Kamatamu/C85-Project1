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
          Position_X();  - Gives you the lander's horizontal position (0 to
   1024) Position_Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES
   (upside-down = 180 degrees)

          SONAR_DIST[];  - Array with distances obtained by sonar. Index
   corresponds to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing
   upward) SONAR_DIST[1] is distance at 10 degrees from vertical SONAR_DIST[2]
   is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from
   vertical

                           if distance is '-1' there is no valid reading. Note
   that updating the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the
   distance to ground in the direction of the lander's main thruster. The laser
   range finder never fails (probably was designed and built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

        - Variables accessible to your 'in flight' computer

          MT_OK		- Boolean, if 1 indicates the main thruster is working
   properly RT_OK		- Boolean, if 1 indicates the right thruster is
   working properly LT_OK		- Boolean, if 1 indicates the left
   thruster is working properly PLAT_X	- X position of the landing platform
          PLAY_Y  - Y position of the landing platform

        - Control of the lander is via the following functions
          (which are noisy!)

          Main_Thruster(double power);   - Sets main thurster power in [0 1], 0
   is off Left_Thruster(double power);	 - Sets left thruster power in [0 1]
          Right_Thruster(double power);  - Sets right thruster power in [0 1]
          Rotate(double angle);	 	 - Rotates module 'angle' degrees
   clockwise (ccw if angle is negative) from current orientation (i.e. rotation
   is not w.r.t. a fixed reference direction).

                                           Note that rotation takes time!


        - Important constants

          G_ACCEL = 8.87	- Gravitational acceleration on Venus
          MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
          RT_ACCEL = 25.0	- Max acceleration provided by right thruster
          LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per
   unit time

        - Functions you need to analyze and possibly change

          * The Lander_Control(); function, which determines where the lander
   should go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander
   is in danger of crashing, and calls control functions to prevent this.

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

          * The code I am providing will land the module assuming nothing goes
   wrong with the sensors and/or controls, both for the 'easy.ppm' and
   'hard.ppm' maps.

          * Failure modes: 0 - Nothing ever fails, life is simple
                           1 - Controls can fail, sensors are always reliable
                           2 - Both controls and sensors can fail (and do!)
                           3 - Selectable failure mode, remaining arguments
   determine failing component(s): 1 - Main thruster 2 - Left Thruster 3 - Right
   Thruster 4 - Horizontal velocity sensor 5 - Vertical velocity sensor 6 -
   Horizontal position sensor 7 - Vertical position sensor 8 - Angle sensor 9 -
   Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main
   thruster, vertical velocity sensor, and angle sensor.

                * Note - while running. Pressing 'q' on the keyboard terminates
   the program.

        * Be sure to complete the attached REPORT.TXT and submit the report as
   well as your code by email. Subject should be 'C85 Safe Landings,
   name_of_your_team'

        Have fun! try not to crash too many landers, they are expensive!

        Credits: Lander image and rocky texture provided by NASA
                 Per Parker spent some time making sure you will have fun!
   thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"

#define FLOATING_TOLERANCE 0.00001f
#define NSAMPLES 12000
#define MAIN_OFFSET 0.0
#define LEFT_OFFSET 90.0
#define RIGHT_OFFSET -90.0
#define HISTORY_LENGTH 64
// Setting a thruster to 0 seems to set it to 2.5% of it's max power
// on average, and every rotate seems to drift a bit by this as well
#define ADDITIVE_BIAS 0.025
// Rotate(), Main/Left/Right_Thruster() all deliver 95% of whatever is
// asked (mean 0.94999, median 0.95000 over 292 samples of all controls)
#define MULTIPLICATIVE_BIAS 0.95
// Rotation rate per frame in degrees.
#define ROTATE_DEG_PER_FRAME (MAX_ROT_RATE * 180.0 / PI)
// When a velocity sensor is broken but its position sensor works, this fraction
// of the position error is given back to the velocity each frame. Without it,
// the predicted velocity slowly drifts.
#define POSITION_TO_VELOCITY_GAIN 0.005
// NEVER tip the firing thruster further than this from straight up. Note that technically,
// the thruster and gravity completely cancel out at ~69 degrees. However, since the angle
// sensor / calculation can be slightly off, we give ourselves a ~10 degree margin. Also 60
// is nice cuz cos(60) = sin(90 - 60) = 0.5
#define MAX_TILT 60.0
// Skip re-rotating for corrections this small
#define ROTATE_TOLERANCE 3.0
// These variances are the cuttoff for determining if a sensor is broken or not.
#define POSITION_VARIANCE_TOLERANCE 1000.0
#define VELOCITY_VARIANCE_TOLERANCE 10.0
#define ANGLE_VARIANCE_TOLERANCE 50.0
// Target horizontal speed for each metre between us and the platform. A higher value
// makes movement quicker but increases the probability of oveshooting. I think we can
// get away with making this higher, maybe 0.7 even
#define TARGET_VX_PER_METRE_FROM_PLATFORM 0.6
// Horizontal acceleration for each m/s we're off the target speed.
#define AX_PER_VX_ERROR 3.6
// Extra upward thrust (m/s^2) for each m/s we're falling faster than the speed limit. Lander_Control
// adds G_ACCEL on top of this to cancel gravity, since thrust() doesn't include gravity
#define AY_PER_VY_ERROR 6
// Sonar does not rotate with the engine. A failed sonar reads all -1, same as
// "nothing in range", so it's only broken if RangeDist sees something close
// while every beam stays -1 for longer than the slowest sonar refresh (surely
// 60 is enough. 60 fps)
#define SONAR_CHECK_RANGE 300.0
#define SONAR_BROKEN_FRAMES 60
// fminmax. This is too small to be a function. Macro way better
#define FMM(x, y) fmax(-(x), fmin((x), (y)))
// Stay upright and stop propulsion sideways, but keep using the
// main thruster to control the descent all the way down.
// ONLY IF MAIN THRUSTER WORKS
#define TOUCHDOWN_HEIGHT 40.0
// Straighten up, cut all thrust and free-fall the rest
// ONLY IF MAIN THRUSTER BROKEN
// Without the main thruster, upright means no lift, so the last phase of the landing is a free fall.
// Contact with the platform happens at py ~ PLAT_Y - 20, so dropping from 36 means falling 16 px (3 m). Rotating upright
// takes ~21 frames and uses < 1 px of height per frame
#define DROP_HEIGHT 36.0
// Don't let go if we're already falling faster than this (m/s). Falling 3 m starting at 4 m/s
// hits at ~8.3 m/s, and starting at 5 m/s hits at ~8.8 m/s. The limit is 10, so we leave ourselves
// ~1 m/s of wiggle room
#define DROP_MAX_FALL_SPEED 5.0
// Within 10 px sideways counts as "over the platform" laterally
#define ABOVE_PLATFORM_TOLERANCE 10.0
// Good Morning Canada. Breaking news! Our state-of-the-art Multi-Billion Dollar Rover keeps
// thrashing for God knows why. Luckily, our greastest minds have come together to figure out issue,
// spending hours around the clock to resolve the issue.
// Professor Plutonium: I find it incredible how aggregate human ingenuity can coalesce into one stupid mass!
// I truly wish there was an antonym for synergy because that's what this is :D
// I'm losing my mind
// In single-thruster mode, only fire the thruster once it is at least this close to the target angle
#define FIRE_ALIGNMENT_TOLERANCE 30.0
// If we have 2 thrusters that can do the job, then only switch thrusters if the other needs this
// much less rotation (left needs 88, right needs 92 make us flip-flop and never finish turning), so
// just pick one and stick with it instead of being indecisive
#define THRUSTER_SWITCH_BIAS 45.0

enum NormalizeType { Nangle, Nthrust };

enum WhichThruster {
    MAIN_THRUSTER,
    LEFT_THRUSTER,
    RIGHT_THRUSTER
};

struct Stats {
    double mean, variance;
};

// A summary of a snapshot of the rover's current state
struct QuasiState {
    double px, py, vx, vy, angle, range;
    double main_cmd, left_cmd, right_cmd;
};

// Ring buffer of the last HISTORY_LENGTH frames (no need to use vector)
struct PastQuasiStates {
    QuasiState states[HISTORY_LENGTH];
    int head = -1;
    int count = 0;

    void push(QuasiState state) {
        head = (head + 1) % HISTORY_LENGTH;
        states[head] = state;
        if (count < HISTORY_LENGTH)
            count++;
    }

    // Retrieves the summaray from 'frame" frames ago
    // 0 = this frame, 1 = last frame, ... NULL if we don't have that many
    // frames yet
    QuasiState *ago(int frames) {
        if (frames < 0 || frames >= count)
            return NULL;
        return states + ((head - frames + HISTORY_LENGTH) % HISTORY_LENGTH);
    }
};

// An adaptive physics engine that predicts where the rover should be.
// Additionally accepts real positions to counteract inaccuracies
//
// BIG NOTE: (0, 0) IS TOP LEFT, SO
// px, py in pixels (py grows DOWNWARD), vx, vy in m/s (vy positive is UP), 1 m
// = S_SCALE px. angle in degrees [0, 360), clockwise. One frame = one T_STEP.
struct PhysicsEngine {

    double px = 0, py = 0, vx = 0, vy = 0, angle = 0;

    // Last command sent to each control. Only the last call in a frame counts.
    double main_cmd = 0, left_cmd = 0, right_cmd = 0;
    // Rotate() is relative and keeps turning over later frames until done. A
    // new call replaces whatever is left, and does that rotation starting from
    // the current angle.
    double rotate_remaining = 0;

    void init(double new_px, double new_py, double new_vx, double new_vy, double new_angle) {
        px = new_px;
        py = new_py;
        vx = new_vx;
        vy = new_vy;
        angle = new_angle;
    }

    void set_rotate(double degrees) {
        rotate_remaining = degrees * MULTIPLICATIVE_BIAS + ADDITIVE_BIAS;
    }

    // Advance one T_STEP using the commands from last frame
    void predict(int mt_ok, int lt_ok, int rt_ok) {
        double step = FMM(ROTATE_DEG_PER_FRAME, rotate_remaining);
        rotate_remaining -= step;
        angle = fmod(angle + step + 360.0, 360.0);

        // Inital accelerations in either direction
        double ax = 0;
        double ay = -G_ACCEL;

        // Add the acceleration components of various thruster pulses
        add_thrust_component(&ax, &ay, mt_ok, MT_ACCEL, main_cmd, MAIN_OFFSET);
        add_thrust_component(&ax, &ay, lt_ok, LT_ACCEL, left_cmd, LEFT_OFFSET);
        add_thrust_component(&ax, &ay, rt_ok, RT_ACCEL, right_cmd, RIGHT_OFFSET);

        // Add the acceleration components to position / velocities
        vx += ax * T_STEP;
        vy += ay * T_STEP;
        px += vx * T_STEP * S_SCALE;
        py -= vy * T_STEP * S_SCALE;
    }

    // If the velocity sensor is broken but we know position, we can use position
    // delta to predct a delta for the velocity
    void correct_px(double measured, int vx_healthy) {
        // Add because +dy is same direction of +dy/dx
        if (!vx_healthy)
            vx += POSITION_TO_VELOCITY_GAIN * (measured - px) / (T_STEP * S_SCALE);
        px = measured;
    }

    void correct_py(double measured, int vy_healthy) {
        // Subtract because +dy is opposite of +dy/dx
        if (!vy_healthy)
            vy -= POSITION_TO_VELOCITY_GAIN * (measured - py) / (T_STEP * S_SCALE);
        py = measured;
    }

    void correct_vx(double measured) {
        vx = measured;
    }

    void correct_vy(double measured) {
        vy = measured;
    }

    void correct_angle(double measured) {
        angle = measured;
    }

  private:
    // Add the acceleration from a thruster pointing at angle + offset (only if it was active)
    void add_thrust_component(
        double *ax,
        double *ay,
        int ok,
        double max_accel,
        double cmd,
        double offset
    ) {
        if (!ok)
            return;
        // accell in m/s
        double accel = max_accel * (ADDITIVE_BIAS + MULTIPLICATIVE_BIAS * cmd);
        double direction = (angle + offset) * PI / 180.0;
        *ax += accel * sin(direction);
        *ay += accel * cos(direction);
    }
};

/*
Abstract:

Any combination of thrusters and sensors can break, so we must be prepared for
everything. Lander_Control attempts to land the thruster. Safety_Override tries
to make sure Lander_Control doesn't do anything stupid that might crash the
rover. But there is a pilot on the rober, so if the pilot wishes, they *might*
override Lander_Control. This means that using a physics simulation as the
ultimate backup might not feasible as we are not only dealing with inputs from
our Lander_Control. So Safety_Override has to try and aide the pilot, allow then
to fly (relatively) freely, but stop them from accidentally crashing into the
ground or leaving the screen. Moreover, Sonar can fail, so we cannot rely solely
on it for safety override. By "overriding", we mean that the Pilot is able to
inject code that runs after Lander_Control but before Safety_Override

However we have our own Deus Ex Machina. Safety_Override runs *after* the pilot
and Lander_Control, so whatever it commands is what actually happens (Yippee).
So we never actually lose control, it's just that the pilot's desire is
ambiguous

So what can we do?

We can have a Robust Agent that handles everything in both Lander_Control and
Safety_Override.

Detecting failures:

Sensor noise is random, so averaging NSAMPLES calls in a single frame gives
basically the true value. We measure that once per sensor and we memoize the
value for the given frame (so subsequent requests in the same frame don't
recalculate their respective average value for a sensor), and if the variance
is too high, the sensor is broken. We only check variance, not mean = 0, as we
are mainly looking for randomness (broken position is random over 0-1024, broken
velocity over -25..25, etc). Thresholds are position 1000, velocity 10, angle 50,
way above healthy and way below broken. Once a sensor is broken, it stays broken.
A failed sonar yields -1 for all indeces, same as "nothing in range", so the sonar
is only broken if RangeDist sees something closer than 300 while every beam stays
as -1 for more than 60 frames (the sonar can take less than 60 frames to refresh).
Averaging only helps working sensors though. The fallbacks we (hopefully) have
are only as good as how often we can validate and correct them to some absolute
truth, since integrating velocity drifts and differentiating position exaggerates
noise.

Fallbacks:

All of these go through the PhysicsEngine. Every frame it predicts the state from
last frame's commands (which the agent records), then overwrites it with every
healthy sensor. So a broken sensor just returns the prediction.

Velocity_X: Use Position_X to calculate it. With no sensor, vx comes purely from
the physics prediction (last vx plus whatever the thrusters added), and the gap
between predicted and measured px adjusts it (POSITION_TO_VELOCITY_GAIN), otherwise
vx will drift.
Position_X: Use Velocity_X to calculate it. The prediction adds up the (true)
velocity every frame, so nothing extra is needed.
Velocity_X and Position_X: What do we do when both fail? Pure prediction. It
drifts a few px per s, as long as we know what thrust was applied. But what if
the pilot has taken control? Lander_Control is always called, and the pilot's
Manual runs between it and Safety_Override, so we can't tell from the calls. We
could tell from the physics though: if predicted and measured velocity disagree
by way more than usual on an axis that still has a sensor, then someone
else fired the thrusters. If the pilot has taken control, we can't read what
thrust they asked for, so the sim breaks, and with no X sensor our px would never
recover. But we don't need to take everything away from them, only what we can't
see. With both X sensors broken (but Y and the angle fine), the main thruster's
thrust is still measurable. The vertical acceleration tells us how hard it fired,
and the angle splits that into x and y. Rotation is seen by the angle sensor. Only
the side thrusters are invisible, since they only thrust along x. So Safety_Override
only takes over the side thrusters, re-issuing our own values every frame in
end_frame(), so we know exactly what was applied, and the pilot keeps the main
thruster and rotation. RangeDist doesn't help much here, pointed
sideways it only gives the distance to the nearest hill.

Velocity_Y: Use Position_Y to calculate it. Same as X, with no sensor, vy comes
purely from the physics prediction (last vy plus whatever the thrusters and
gravity added), and the gap between predicted and measured py adjusts it
(POSITION_TO_VELOCITY_GAIN). NOTE ON THE SIGNS: Position_Y increases DOWNWARD
but Velocity_Y is POSITIVE UP, so py moves by -vy, and a py that's further down
than predicted means vy was too low (MUCHO IMPORTANTE).
Position_Y: Use Velocity_Y to calculate it. Same as X, the prediction adds up the
(true) velocity every frame (with the sign flip), so nothing extra is needed.
Velocity_Y and Position_Y: What do we do when both fail? Pure prediction again,
but Y has something absolute to rely on. While upright, RangeDist points straight
down, so it's our height above the ground directly below us. The ground is not
uniform though. When Tilted, the beam hits the ground off to the side, and on a
slope that spot is higher or lower than what's below us, so RangeDist * cos(angle) is
only right over flat ground. And as we move sideways the ground changes under us,
so RangeDist changes even if our height doesn't. RangeDist isn't noisy, but it's
rounded to whole px, so taking how fast it changes gives a terrible vy anyway. 
Better to keep vy coming from the prediction and use the height to slowly adjust it,
like POSITION_TO_VELOCITY_GAIN, but only when over flat ground we know, or while
we're barely moving sideways. To get Position_Y itself we need the ground height
below us. Over the platform that's just PLAT_Y, and it's flat, which we can use.
Elsewhere we remember ground[x] = Position_Y + height (Y increases downward) from
while Position_Y still worked, which also tells us the ground's shape, so we can
correct for it. If we're not upright (i.e. the main thruster is die), RangeDist
points sideways and we fall back to the physics sim alone. Same idea as X for the pilot.
Upright, the main thruster only accelerates along y, which nothing measures now, so Safety_Override
takes over the main thruster and the pilot keeps the side thrusters and rotation,
since the X sensors still see the side thrusters' thrust.

Angle: This is the most tedious part, since there is no single other component
that we can use to find the angle. Instead, we will use a combination. We can
have some sort of internal physics that using a past frame's vx/xy and px/py to
determine ax and ay and using the previous frames active thrusters, figure out
what angle we must be at for our current variables to be possible. This only
works while thrusting though, as with no thrust, the only acceleration is
gravity, which says nothing about angle, so the cleanest way is a short main
thruster pulse. The direction of the change in velocity (minus gravity) is the
thrust direction. Rotate never breaks, so between pulses, we track the angle by
adding up our own Rotate commands, which is what the PhysicsEngine does now.
Every Rotate(d) keeps turning over later frames until done, and a new call
replaces what's left. With that, a broken angle stays within 1.5 ~ 2 degrees,
which is enough on its own unless everything else is broken too. In that case,
a 1 degree error tilts the main thrust enough to drift us sideways. We can also
sweep a rotation and watch RangeDist, over flat ground the shortest reading is
straight down. But what if the pilot overrides Lander_Control? If the pilot
rotates the rover, we lose track of the angle, so if the angle sensor is broken
we disallow any rotations from the pilot. Then the angle only changes through our
own Rotate calls. Every sensor is guaranteed to work on the first frame, so the
PhysicsEngine starts from them.

Sonar: This is probably the reason why RangeDist exists and cannot fail. A
failed sonar yields all -1, and the sonar is world-fixed (SONAR_DIST[18] is always
straight down, whatever our angle). RangeDist points out of our bottom, i.e. at
angle + 180 in the world, so right now when the sonar breaks we put RangeDist
into the beam it's pointing along and leave the rest -1. That's enough for now,
but we can still fly sideways into hills we can't see. So once in a while, we will
sweep RangeDist around to rebuild a sonar map. Specifcally:
Rotate, record a reading at each angle, turn each one into a ground point,
and work out fake beams from our current position every frame. A full 360 is only
84 frames (2pi / 0.075, 0.42 s) and costs at most ~4 m/s even with no thrust, but
a partial sweep (+-60, and 90 degrees toward where we're going) plus remembering
the terrain we've already seen is definently enough to work with. Safety_Override
runs last, so its Rotate calls takes priority during the sweep, and we give back
control to the Lander_Control / Manual after. RangeDist itself isn't noisy at all,
just rounded to whole px, and it reads -1 when its beam hits nothing (off the map).

Thrusters: The controllers don't touch thrusters directly anymore. They ask for
a general thrust(ax, ay) in world coords terms, and the agent figures out how with
whatever still works. If the main thruster and the side thruster we need both
work, we stay upright and use them directly. Otherwise we point ONE working
thruster along the direction of thrust, whichever needs the least rotation (so if the main
thruster fails we end up on a side thruster at ~90 degrees, and if that one
fails too we just end up on the third). If a side thruster fails, then we can still
obtain a horizontal component by tilting the main thruster. A tilted thruster
also accelerates us up, so its power is capped to never lift more than asked for
(at least enough for sustained altitude), otherwise accelerating sideways makes us
climb off the screen. The tilt is the angle of the requested propulsion from straight
up, atan2(ax, ay). But when little or no lift is asked for (e.g. ay = 0 while
descending), even a tiny sideways thrust points almost straight sideways.
atan2(0.2, 0) is 90 degrees, so we'd tip over for a correction that's basically
0. So we use max(ay, G_ACCEL) instead of ay, so we treat every request as
at least hover thrust plus the sideways bit. Then the tilt increases with how much
sideways thrust we actually want.
We must be within the landing constraints though (< 15 degrees, < 10 m/s), so
close above the platform we stay upright no matter what. With the main thruster
broken we can't land on a side thruster, since upright means no lift. The
solution is to hover low on the side thruster, rotate upright, then free fall the
rest of the way. From rest, h < 10^2 / (2 * 8.87) ~ 5.6 m ~ 28 px, and rotating
upright (pi / 2) only takes ~21 frames (< 1 px of falling). So we let go at
DROP_HEIGHT (36 px above PLAT_Y, contact happens ~21 px above it) and commit to it.
Height comes from our position estimate, and the sonar is world-fixed so SONAR_DIST[18]
works at any tilt. Only if Position_Y and the sonar are both broken do we need
RangeDist, which points sideways while on a side thruster. A side thruster gives
25 vs 8.87 gravity, so we can still hold altitude up to acos(8.87 / 25) ~ 69
degrees off the thruster's vertical, which points RangeDist ~21 degrees from
straight down. This accelerates us sideways, so we should only "look down" briefly.

We should try and resist the potentially malicious actions of the pilot to the
best of our ability, but if too many sensors are broken, then we might just have
to yield power to the Malediktator. Since Safety_Override always has the final
say, yielding is a choice we make per failure state. The plan is, if both sensors
on an axis are broken, or the angle sensor is broken, Safety_Override takes over
only the controls we can't see the effect of and the pilot keeps the rest. With
healthy sensors the pilot flies freely, and Safety_Override only steps in near
danger.

Plan:
For the Rover to ever crash, the following must be true:

(None of the thrusters are working) OR
(Main thruster is broken AND height can't be measured near touchdown, i.e.
    Position_Y, the sonar AND the RangeDist glance all fail us) OR
(We chose to yield to the pilot in a degraded state)

Otherwise, the rover should be able to land or let the pilot navigate freely

*/
class RobustAgent {

  public:
    int frame = 0;
    struct {
        int frame = -1;
        Stats stats;
    } memoized[6];
    double sonar_dist[36];
    int sonar_silent_frames = 0;
    int current_thruster = -1;

    PhysicsEngine physics;
    PastQuasiStates history;

    // last thrust() request, so Safety_Override can override one half
    double prev_ax = 0, prev_ay = 0;

    unsigned int mt_broken : 1;
    unsigned int rt_broken : 1;
    unsigned int lt_broken : 1;
    unsigned int dropping : 1;
    unsigned int vx_broken : 1;
    unsigned int vy_broken : 1;
    unsigned int px_broken : 1;
    unsigned int py_broken : 1;
    unsigned int angle_broken : 1;
    unsigned int sonar_broken : 1;

    // Rotate and RangeDist never break

    RobustAgent() {
        mt_broken = 0;
        rt_broken = 0;
        lt_broken = 0;
        dropping = 0;
        vx_broken = 0;
        vy_broken = 0;
        px_broken = 0;
        py_broken = 0;
        angle_broken = 0;
        sonar_broken = 0;
    }


    // The physics state is updated once per frame in update_frame(): predicted
    // from last frame's commands, then overwritten by every healthy sensor. So
    // a healthy sensor returns its averaged reading, and a broken one returns
    // the prediction.
    //
    // Todo: Position_Y fallback could also use RangeDist / sonar against PLAT_Y
    // or remembered terrain,
    //       and Angle could be recalibrated with thrust pulses. For now they're
    //       prediction only.

    double velocity_x() {
        return physics.vx;
    }

    double velocity_y() {
        return physics.vy;
    }

    double position_x() {
        return physics.px;
    }

    double position_y() {
        return physics.py;
    }

    double angle() {
        return physics.angle;
    }

    void rotate(double degrees) {
        physics.set_rotate(degrees);
        Rotate(degrees);
    }

    void begin_frame() {
        frame++;
        update_frame();
    }

    void end_frame() {}

    // Stop thrusting and straighten up. Used for the final free fall when the main thruster is broken
    void go_upright() {
        rotate_absolute(0.0);
        main_thruster(0.0);
        left_thruster(0.0);
        right_thruster(0.0);
    }


    void thrust(double ax, double ay) {
        // Thrusters can only ever propell. It doesn't make sense to ask a thruster to thrust and yield
        // negative acceleration in the direction that is thrusting. In general, we let gravity do the work.
        // We aren't Sebastian.
        ay = fmax(ay, 0.0);
        prev_ax = ax;
        prev_ay = ay;
        if (mt_broken != 1 && PLAT_Y - physics.py < TOUCHDOWN_HEIGHT)
            ax = 0.0;
        double angle = physics.angle;
        WhichThruster side = (ax >= 0) ? LEFT_THRUSTER : RIGHT_THRUSTER;
        int side_ok = (side == LEFT_THRUSTER) ? lt_broken != 1 : rt_broken != 1;
        // If the main thruster is not broken and the truster that can accelerate us in the x that we want to go to
        // is available, then just do that and return
        if (mt_broken != 1 && (side_ok || fabs(ax) < FLOATING_TOLERANCE)) {
            // Rotation does take time though
            rotate_absolute(0.0);
            fire(MAIN_THRUSTER, ay, ax, angle);
            fire(LEFT_THRUSTER, ay, ax, angle);
            fire(RIGHT_THRUSTER, ay, ax, angle);
            return;
        }
        // Otherwise tilt as if least hover thrust were asked for, so a tiny sideways push gives a tiny tilt
        // (atan2(0.2, 0) would be 90 degrees for almost nothing)
        double push = atan2(ax, fmax(ay, G_ACCEL)) * 180.0 / PI;
        // Limit how much we can tilt to apply this force
        push = FMM(MAX_TILT, push);
        WhichThruster options[3] = { MAIN_THRUSTER, LEFT_THRUSTER, RIGHT_THRUSTER };
        unsigned int broken[3] = { mt_broken, lt_broken, rt_broken };
        // Selection process for best thruster
        int best = -1;
        double best_turn = 1e9;
        for (int i = 0; i < 3; i++) {
            if (broken[i] == 1)
                continue;
            // The angle the i'th thruster would need to turn to propell us in the direction we want
            double turn = fabs(find_min_travel_angle(push - push_offset(options[i]), angle));
            // Stick with the thruster we're already using unless another is clearly better
            // We incentivise the rover to stay with the current thruster by making it's rotation
            // seem cheaper than it actually is. Only if another available thruster requires significantly
            // less rotation do we switch
            if (i == current_thruster)
                turn -= THRUSTER_SWITCH_BIAS;
            // We are looking for the thruster that we have to turn the least for
            if (turn < best_turn) {
                best_turn = turn;
                best = i;
            }
        }
        // Set all unnecessary thrusters to 0
        for (int i = 0; i < 3; i++)
            if (i != best)
                set_power(options[i], 0.0);
        // If no thruster was selected, then it means no thrusters work. We literally cannot succeed,
        // it is physically impossible. Resign.
        if (best == -1)
            return;
        current_thruster = best;
        // Rotate to the direction that we want to thrust in
        double target = push - push_offset(options[best]);
        rotate_absolute(target);
        // Fire only once we're mostly facing the right way, otherwise part of the propulsion goes
        // sideways while we're still turning
        if (fabs(find_min_travel_angle(target, angle)) < FIRE_ALIGNMENT_TOLERANCE)
            fire(options[best], ay, ax, angle);
        else
            set_power(options[best], 0.0);
    }

  private:

    // Rotate toward an absolute angle unless we're already close enough
    void rotate_absolute(double target) {
        double diff = find_min_travel_angle(target, physics.angle);
        if (fabs(diff) > ROTATE_TOLERANCE)
            rotate((diff - ADDITIVE_BIAS) / MULTIPLICATIVE_BIAS);
    }

    // Fire thruster t with only the part of (ax, ay) it can deliver pointing where it points
    // right now (zero if it's pointing the wrong way while we're still turning)
    void fire(WhichThruster t, double ay, double ax, double angle) {
        double dir = (angle + push_offset(t)) * PI / 180.0;
        // A tilted thruster also thrusts up. Don't let that lift more than was asked for
        // (at least enough to hover), or thrusting sideways makes us climb
        double up = cos(dir);
        double power = ax * sin(dir) + ay * up;
        // If this thruster propells up at all, limit how much it may lift
        if (up > 0.0) {
            // All this technically isn't necessary, but I noticed it helps reduce drift up when hover.
            // How much lift we allow when the controller asked for less (ay < G_ACCEL, "let us fall").
            // Hover, so sideways pushes still work. But if we're already rising, hover would keep us
            // rising, so we allow a bit less than hover, which makes the upward speed die out
            double lift_allowance = G_ACCEL - AY_PER_VY_ERROR * fmax(0.0, physics.vy);
            // Asking for more lift than that (ay > lift_allowance means to climb) is always honoured.
            // The cap is on the upward part (power * up), hence dividing by up
            power = fmin(power, fmax(ay, lift_allowance) / up);
        }
        set_power(t, fmax(0.0, power) / max_accel(t));
    }

    void set_power(WhichThruster t, double fraction) {
        if (t == MAIN_THRUSTER)
            main_thruster(fraction);
        else if (t == LEFT_THRUSTER)
            left_thruster(fraction);
        else
            right_thruster(fraction);
    }

    // This function calculates the angle to make inside our call to
    // rotate from one angle to another.
    double find_min_travel_angle(double required_angle, double state_angle) {
            double diff = required_angle - state_angle;
        diff = fmod(diff, 360.0);
        if (diff <= -180.0)
            diff += 360.0;
        else if (diff > 180.0)
            diff -= 360.0;
        return diff;
    }

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

    // Every control goes through these so the physics engine knows what was
    // applied. Only the last call in a frame counts, so each call overwrites
    // the recorded value.
    void main_thruster(double power) {
        physics.main_cmd = normalize_value(power, Nthrust);
        Main_Thruster(physics.main_cmd);
    }

    void left_thruster(double power) {
        physics.left_cmd = normalize_value(power, Nthrust);
        Left_Thruster(physics.left_cmd);
    }

    void right_thruster(double power) {
        physics.right_cmd = normalize_value(power, Nthrust);
        Right_Thruster(physics.right_cmd);
    }
    
    // Once per frame: predict, detect failures, correct with healthy sensors,
    // remember
    void update_frame() {
        if (frame > 1)
            physics.predict(mt_broken != 1, lt_broken != 1, rt_broken != 1);

        detect_failures();

        // Every sensor works on the first frame
        if (frame == 1) {
            physics.init(
                Robust(Position_X).mean, Robust(Position_Y).mean,
                Robust(Velocity_X).mean, Robust(Velocity_Y).mean,
                Robust(Angle).mean
            );
        } else {
            // We must calculate velocities first, so that correct_px/py knows whether
            // they need to nudge them
            if (vx_broken != 1)
                physics.correct_vx(Robust(Velocity_X).mean);
            if (vy_broken != 1)
                physics.correct_vy(Robust(Velocity_Y).mean);
            if (px_broken != 1)
                physics.correct_px(Robust(Position_X).mean, vx_broken != 1);
            if (py_broken != 1)
                physics.correct_py(Robust(Position_Y).mean, vy_broken != 1);
            if (angle_broken != 1)
                physics.correct_angle(Robust(Angle).mean);
        }

        QuasiState state = {
            .px = physics.px,
            .py = physics.py,
            .vx = physics.vx,
            .vy = physics.vy,
            .angle = physics.angle,
            .range = RangeDist(),
            .main_cmd = physics.main_cmd,
            .left_cmd = physics.left_cmd,
            .right_cmd = physics.right_cmd
        };
        history.push(state);
        // If sonar is not broken
        if (sonar_broken != 1) {
            for (int i = 0; i < 36; i++)
                sonar_dist[i] = SONAR_DIST[i];
        } else {
            // Reset the data (but do we really want to do this?)
            for (int i = 0; i < 36; i++)
                sonar_dist[i] = -1;
            double range = RangeDist();
            if (range > 0) {
                int beam = lround((physics.angle + 180.0) / 10.0) % 36;
                sonar_dist[beam] = range;
            }
        }
    }

    // Once something is broken, it stays broken, so broken sensors aren't
    // sampled again
    void detect_failures() {
        mt_broken = !MT_OK;
        rt_broken = !RT_OK;
        lt_broken = !LT_OK;
        if (px_broken != 1 &&
            Robust(Position_X).variance >= POSITION_VARIANCE_TOLERANCE)
            px_broken = 1;
        if (py_broken != 1 &&
            Robust(Position_Y).variance >= POSITION_VARIANCE_TOLERANCE)
            py_broken = 1;
        if (vx_broken != 1 &&
            Robust(Velocity_X).variance >= VELOCITY_VARIANCE_TOLERANCE)
            vx_broken = 1;
        if (vy_broken != 1 &&
            Robust(Velocity_Y).variance >= VELOCITY_VARIANCE_TOLERANCE)
            vy_broken = 1;
        if (angle_broken != 1 &&
            Robust(Angle).variance >= ANGLE_VARIANCE_TOLERANCE)
            angle_broken = 1;
        if (sonar_broken != 1)
            detect_sonar_failure();
    }

    // Sonar is world-fixed (SONAR_DIST[18] is always straight down) and sees
    // ~420 px. RangeDist can also be -1 when its beam hits nothing (e.g.
    // pointing off the map).
    void detect_sonar_failure() {
        double range = RangeDist();
        int all_invalid = 1;
        for (int i = 0; i < 36; i++) {
            if (SONAR_DIST[i] > -1) {
                all_invalid = 0;
                break;
            }
        }
        if (all_invalid && range > 0 && range < SONAR_CHECK_RANGE)
            sonar_silent_frames++;
        else
            sonar_silent_frames = 0;
        if (sonar_silent_frames > SONAR_BROKEN_FRAMES)
            sonar_broken = 1;
    }

    int sensor_to_number(double (*Sensor)(void)) {
        if (Sensor == Position_X)
            return 0;
        if (Sensor == Position_Y)
            return 1;
        if (Sensor == Velocity_X)
            return 2;
        if (Sensor == Velocity_Y)
            return 3;
        if (Sensor == Angle)
            return 4;
        if (Sensor == RangeDist)
            return 5;
        return -1;
    }

    double normalize_value(double value, NormalizeType type) {
        if (type == Nangle) {
            value = fmod(value, 360.0);
            if (value > 180.0)
                value -= 360.0;
            else if (value <= -180.0)
                value += 360.0;
        } else if (type == Nthrust) {
            value = fmin(fmax((value - ADDITIVE_BIAS) / MULTIPLICATIVE_BIAS, 0.0), 1.0);
        }
        return value;
    }

    Stats Robust(double (*Sensor)(void)) {
        int idx = sensor_to_number(Sensor);
        if (idx != -1 && memoized[idx].frame == frame)
            return memoized[idx].stats;
        double sum = 0;
        double sumsq = 0;
        // Angle samples are taken relative to a first reading so they don't
        // wrap (e.g. 359 vs 1, or 179 vs -179 when upside down)
        double reference = (Sensor == Angle) ? Sensor() : 0;
        for (int i = 0; i < NSAMPLES; i++) {
            double value;
            if (Sensor == Angle)
                value = normalize_value(Sensor() - reference, Nangle);
            else
                value = Sensor();
            sum += value;
            sumsq += value * value;
        }
        Stats stats = {.mean = sum / NSAMPLES, .variance = sumsq / NSAMPLES};
        // Variance = sum of squares / N - mean squared
        stats.variance -= stats.mean * stats.mean;
        if (Sensor == Angle) {
            stats.mean = fmod(stats.mean + reference, 360.0);
            if (stats.mean < 0)
                stats.mean += 360.0;
        }
        if (idx != -1) {
            memoized[idx].frame = frame;
            memoized[idx].stats = stats;
        }
        return stats;
    }
};

static RobustAgent agent = RobustAgent();

void Lander_Control(void) {
    /*
      This is the main control function for the lander. It attempts
      to bring the ship to the location of the landing platform
      keeping landing parameters within the acceptable limits.

      How it works:

      - First, if the lander is rotated away from zero-degree angle,
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

            For example: Write a function Velocity_X_robust()
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

    agent.begin_frame();
    // Without the main thruster we can't be upright while simultaenously having lift, so once
    // we're low and slow over the platform, straighten up and fall the last few px
    if (agent.mt_broken) {
        double height = PLAT_Y - agent.position_y();
        if (
            !agent.dropping &&
            fabs(PLAT_X - agent.position_x()) < ABOVE_PLATFORM_TOLERANCE &&
            height < DROP_HEIGHT &&
            agent.velocity_y() > -DROP_MAX_FALL_SPEED
        )
            agent.dropping = 1;
        if (agent.dropping)
            return agent.go_upright();
    }


    double VXlim;
    double VYlim;

    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.
    if (fabs(agent.position_x() - PLAT_X) > 200)
        VXlim = 25;
    else if (fabs(agent.position_x() - PLAT_X) > 100)
        VXlim = 15;
    else
        VXlim = 5;

    if (PLAT_Y - agent.position_y() > 200)
        VYlim = -20;
    else if (PLAT_Y - agent.position_y() > 100)
        VYlim = -10; // These are negative because they
    else
        VYlim = -4; // limit descent velocity

    // IMPORTANT NOTE: The following section is a DEVIATION from the original
    // code as it tries to keep the rover upright, however this isn't the right
    // thing to do when the main thruster fails. So we do NOT always rotate back
    // to 0. Additionally, now we must use the thrust API

    // Ensure we will be OVER the platform when we land. Skip this once we're already over it.
    // Right above the platform, vx is ~0, so distance / vx explodes
    double plat_dx = fabs(PLAT_X - agent.position_x());
    if (plat_dx > ABOVE_PLATFORM_TOLERANCE &&
        plat_dx / fabs(agent.velocity_x()) >
        1.25 * fabs(PLAT_Y - agent.position_y()) / fabs(agent.velocity_y()))
        VYlim = 0;

    
    // For the horizontal acceleration, aim for a velocity toward the platform.
    // It shoudl get slower as we get close, and be capped at VXlim,
    // and thrust in proportion to how far off that velocity we are
    double dx = (PLAT_X - agent.position_x()) / S_SCALE; // # of meters to travel
    // Aim to mive towards the platform at +0.5m/s for every meter of distance
    double vx_target = FMM(VXlim, TARGET_VX_PER_METRE_FROM_PLATFORM * dx);
    double ax = AX_PER_VX_ERROR * (vx_target - agent.velocity_x());
    // For the vertical acceleration, we must cancel gravity, plus thrust proportionately
    // to how far below the speed limit we're falling
    double ay = G_ACCEL + AY_PER_VY_ERROR * (VYlim - agent.velocity_y());
    agent.thrust(ax, ay);
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

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag = agent.velocity_x() * agent.velocity_x();
    Vmag += agent.velocity_y() * agent.velocity_y();

    DistLimit = fmax(75, Vmag);

    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
    if (fabs(PLAT_X - agent.position_x()) < 150 &&
        fabs(PLAT_Y - agent.position_y()) < 150
    )
        return agent.end_frame();

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.
    dmin = 1000000;
    if (agent.velocity_x() > 0) {
        for (int i = 5; i < 14; i++)
            if (agent.sonar_dist[i] > -1 && agent.sonar_dist[i] < dmin)
                dmin = agent.sonar_dist[i];
    } else {
        for (int i = 22; i < 32; i++)
            if (agent.sonar_dist[i] > -1 && agent.sonar_dist[i] < dmin)
                dmin = agent.sonar_dist[i];
    }
    // Determine whether we're too close for comfort. There is a reason
    // to have this distance limit modulated by horizontal speed...
    // what is it?
    if (dmin < DistLimit * fmax(.25, fmin(fabs(agent.velocity_x()) / 5.0, 1))) {
        // Too close to a surface in the horizontal direction.
        // Thrust against the direction of motion, keeping whatever vertical thrust was asked for.
        // thrust() handles the orientation, so there's no need to straighten up first
        agent.thrust(agent.velocity_x() > 0 ? -LT_ACCEL : LT_ACCEL, agent.prev_ay);
    }

    // Vertical direction
    dmin = 1000000;
    if (agent.velocity_y() > 5) {
        // Mind this! there is a reason for it...
        for (int i = 0; i < 5; i++)
            if (agent.sonar_dist[i] > -1 && agent.sonar_dist[i] < dmin)
                dmin = agent.sonar_dist[i];
        for (int i = 32; i < 36; i++)
            if (agent.sonar_dist[i] > -1 && agent.sonar_dist[i] < dmin)
                dmin = agent.sonar_dist[i];
    } else {
        for (int i = 14; i < 22; i++)
            if (agent.sonar_dist[i] > -1 && agent.sonar_dist[i] < dmin)
                dmin = agent.sonar_dist[i];
    }
    if (dmin < DistLimit) {
        // Too close to a surface in the vertical direction.
        // Stop firing up if we're already climbing, otherwise go full throttle.
        // When braking against the ground, Do not do any sideways thrust, so a single thruster
        // points straight up instead of tilting and wasting half its thrust. If we're already climbing,
        // then we just cut thrust, and the sideways request can stay
        if (agent.velocity_y() > 2.0)
            agent.thrust(agent.prev_ax, 0.0);
        else
            agent.thrust(0.0, MT_ACCEL);

    }

    return agent.end_frame();
}