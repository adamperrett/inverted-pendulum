//
//  bkout.c
//  BreakOut
//
//  Created by Steve Furber on 26/08/2016.
//  Copyright © 2016 Steve Furber. All rights reserved.
//
// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Spin 1 API includes
#include <spin1_api.h>

// Common includes
#include <debug.h>

// Front end common includes
#include <data_specification.h>
#include <simulation.h>
#include "random.h"
#include <math.h>

#include <recording.h>

//----------------------------------------------------------------------------
// Macros
//----------------------------------------------------------------------------

// Frame delay (ms)
//#define reward_delay 200 //14//20
/*
    number of bins for current angle of the pole
    number of bins for the force to be applied or number of spikes per tick equals a force
    mass of the cart
    mass of the pole
    initial starting angle
    velocity of the cart
    velocity of the pendulum
    base rate for the neurons to fire in each bin
    each spike equals a change in force to be applied (what is that amount)
    receptive field of each bin
    update model on each timer tick and on each spike received, or number of spikes per tick equals a force

    add option to rate (increased poisson P()) code and rank code
*/

//----------------------------------------------------------------------------
// Enumerations
//----------------------------------------------------------------------------
typedef enum
{
  REGION_SYSTEM,
  REGION_PENDULUM,
  REGION_RECORDING,
  REGION_DATA,
} region_t;

typedef enum
{
  SPECIAL_EVENT_REWARD,
  SPECIAL_EVENT_NO_REWARD,
  SPECIAL_EVENT_MAX,
} special_event_t;

typedef enum // forward will be considered positive motion
{
  BACKWARD_MOTOR  = 0x0,
  FORWARD_MOTOR  = 0x1,
} arm_key_t;

//----------------------------------------------------------------------------
// Globals
//----------------------------------------------------------------------------

static uint32_t _time;

//! Should simulation run for ever? 0 if not
static uint32_t infinite_run;

const int max_number_of_arms = 8;

uint32_t *arm_probabilities;

mars_kiss64_seed_t kiss_seed;

int number_of_arms;

int rand_seed;

int arm_choices[8] = {0};

int32_t current_score = 0;
int32_t best_arm = -1;
bool chose_well = false;
int32_t reward_based = 1;
int32_t correct_pulls = 0;

// experimental constraints and variables
float current_time = 0;
float max_motor_force = 10;
float min_motor_force = -10;
float motor_force = 0;
float force_increment = 0;
float cart_position = 0;
float cart_velocity = 0; // angular
float cart_acceleration = 0; // angular
int max_pole_angle = 36;
int min_pole_angle = -36;
float pole_angle = 0;
float pole_velocity = 0;
float pole_acceleration = 0;

int max_firing_rate = 20;
int encoding_scheme = 0; // 0: rate, 1: time, 2: rank (replace with type def

// experimental parameters
float half_pole_length = 0.5; // m
float gravity = 9.8; // m/s^2
float mass_cart = 1; // kg
float mass_pole = 0.1; // kg
float friction_cart_on_track = 0.0005; // coefficient of friction
float friction_pole_hinge = 0.000002; // coefficient of friction

uint32_t reward_delay;

//! How many ticks until next frame
static uint32_t tick_in_frame = 0;

//! The upper bits of the key value that model should transmit with
static uint32_t key;

//! the number of timer ticks that this model should run for before exiting.
uint32_t simulation_ticks = 0;
uint32_t score_change_count=0;

//----------------------------------------------------------------------------
// Inline functions
//----------------------------------------------------------------------------
static inline void add_reward()
{
  spin1_send_mc_packet(key | (SPECIAL_EVENT_REWARD), 0, NO_PAYLOAD);
  io_printf(IO_BUF, "Got a reward\n");
  current_score++;
}

static inline void add_no_reward()
{
  spin1_send_mc_packet(key | (SPECIAL_EVENT_NO_REWARD), 0, NO_PAYLOAD);
  io_printf(IO_BUF, "No reward\n");
  current_score--;
}

void resume_callback() {
    recording_reset();
}

//void add_event(int i, int j, colour_t col, bool bricked)
//{
//  const uint32_t colour_bit = (col == COLOUR_BACKGROUND) ? 0 : 1;
//  const uint32_t spike_key = key | (SPECIAL_EVENT_MAX + (i << 10) + (j << 2) + (bricked<<1) + colour_bit);
//
//  spin1_send_mc_packet(spike_key, 0, NO_PAYLOAD);
//  io_printf(IO_BUF, "%d, %d, %u, %08x\n", i, j, col, spike_key);
//}

static bool initialize(uint32_t *timer_period)
{
    io_printf(IO_BUF, "Initialise bandit: started\n");

    // Get the address this core's DTCM data starts at from SRAM
    address_t address = data_specification_get_data_address();

    // Read the header
    if (!data_specification_read_header(address))
    {
      return false;
    }
    /*
    simulation_initialise(
        address_t address, uint32_t expected_app_magic_number,
        uint32_t* timer_period, uint32_t *simulation_ticks_pointer,
        uint32_t *infinite_run_pointer, int sdp_packet_callback_priority,
        int dma_transfer_done_callback_priority)
    */
    // Get the timing details and set up thse simulation interface
    if (!simulation_initialise(data_specification_get_region(REGION_SYSTEM, address),
    APPLICATION_NAME_HASH, timer_period, &simulation_ticks,
    &infinite_run, 1, NULL))
    {
      return false;
    }
    io_printf(IO_BUF, "simulation time = %u\n", simulation_ticks);


    // Read breakout region
    address_t breakout_region = data_specification_get_region(REGION_PENDULUM, address);
    key = breakout_region[0];
    io_printf(IO_BUF, "\tKey=%08x\n", key);
    io_printf(IO_BUF, "\tTimer period=%d\n", *timer_period);

    //get recording region
    address_t recording_address = data_specification_get_region(
                                       REGION_RECORDING,address);
    // Setup recording
    uint32_t recording_flags = 0;
    if (!recording_initialize(recording_address, &recording_flags))
    {
       rt_error(RTE_SWERR);
       return false;
    }

    address_t arms_region = data_specification_get_region(REGION_DATA, address);
    encoding_scheme = arms_region[0];
    number_of_arms = arms_region[1];
//    rand_seed = arms_region[2];
    kiss_seed[0] = arms_region[2];
    kiss_seed[1] = arms_region[3];
    kiss_seed[2] = arms_region[4];
    kiss_seed[3] = arms_region[5];
    reward_based = arms_region[6];
    arm_probabilities = (uint32_t *)&arms_region[7];
//    double arm_probabilities[10] = {0}
//    for (int i=1, i<number_of_arms, i=i+1){
//        io_printf(IO_BUF, "converting arm prob %d, stage \n", temp_arm_probabilities[i] i)
//        arm_probabilities[i] = (double)temp_arm_probabilities[i] / 1000.0
//        io_printf(IO_BUF, "probs after = %d\n", arm_probabilities)
//    }
    validate_mars_kiss64_seed(rand_seed);
//    srand(rand_seed);
    //TODO check this prints right, ybug read the address
    io_printf(IO_BUF, "r1 %d\n", (uint32_t *)arms_region[0]);
    io_printf(IO_BUF, "r2 %d\n", (uint32_t *)arms_region[1]);
    io_printf(IO_BUF, "rand3. %d\n", (uint32_t *)arms_region[2]);
    io_printf(IO_BUF, "rand3 0x%x\n", (uint32_t *)arms_region[3]);
    io_printf(IO_BUF, "r4 0x%x\n", arms_region[3]);
    io_printf(IO_BUF, "r5 0x%x\n", arm_probabilities);
    io_printf(IO_BUF, "r6 %u\n", arm_probabilities[0]);
    io_printf(IO_BUF, "r6d %d\n", arm_probabilities[0]);
    io_printf(IO_BUF, "r7 %u\n", arm_probabilities[1]);
    io_printf(IO_BUF, "r7d %d\n", arm_probabilities[1]);
    io_printf(IO_BUF, "re %d\n", reward_based);
//    io_printf(IO_BUF, "r6 0x%x\n", *arm_probabilities);
//    io_printf(IO_BUF, "r6 0x%x\n", &arm_probabilities);

    force_increment = (float)((max_motor_force - min_motor_force) / (float)100);

    int highest_prob = 0;
    for(int i=0; i<number_of_arms; i=i+1){
        if(arm_probabilities[i] > highest_prob){
            best_arm = i;
            highest_prob = arm_probabilities[i];
        }
    }

    io_printf(IO_BUF, "Initialise: completed successfully\n");
    io_printf(IO_BUF, "best arm = %d with prob %d\n", best_arm, highest_prob);

    current_time = 0;
    return true;
}

bool was_there_a_reward(){
    int choice = -1; //mars_kiss64_seed(kiss_seed) % number_of_arms;
//    int choice = rand() % number_of_arms;
    int highest_value = 0;
    int min_spikes = 100000000;
    for(int i=0; i<number_of_arms; i=i+1){
        if(arm_choices[i] < min_spikes){
            min_spikes = arm_choices[i];
        }
    }
    for(int i=0; i<number_of_arms; i=i+1){
        arm_choices[i] = arm_choices[i] - min_spikes;
    }
    if(arm_choices[0] > highest_value){
        choice = 0;
        highest_value = arm_choices[0];
    }
//    io_printf(IO_BUF, "0 was spiked %d times, prob = %u\n", arm_choices[0], arm_probabilities[0]);
    arm_choices[0] = 0;
    for(int i=1; i<number_of_arms; i=i+1){
        if (arm_choices[i] >= highest_value && arm_choices[i] != 0){
            if(arm_choices[i] == highest_value){
                if (mars_kiss64_seed(kiss_seed) % 2 == 0){
//                if (rand() % 2 == 0){
                    choice = i;
                    highest_value = arm_choices[i];
                }
            }
            else{
                choice = i;
                highest_value = arm_choices[i];
            }
        }
//        io_printf(IO_BUF, "%d was spiked %d times, prob = %u\n", i, arm_choices[i], arm_probabilities[i]);
        arm_choices[i] = 0;
    }
//    io_printf(IO_BUF, "choice was %d and best arm was %d, score is %d, highest value: %d", choice, best_arm, current_score, highest_value);
    if(choice == best_arm){
        correct_pulls++;
    }
    else{
        correct_pulls--;
    }
    if(highest_value == 0){
        return false;
    }
    else{
        uint32_t probability_roll;
    //    double max = RAND_MAX;
//        io_printf(IO_BUF, "rand = %d, max = %d\n", rand_no, RAND_MAX);
        probability_roll = mars_kiss64_seed(kiss_seed);
    //    probability_roll = rand();
//        io_printf(IO_BUF, "prob_roll = %u\n", probability_roll);
//        io_printf(IO_BUF, "roll was %u and prob was %u, max = %u %d\n", probability_roll, arm_probabilities[choice], RAND_MAX, RAND_MAX);
        if(probability_roll < arm_probabilities[choice]){
    //        io_printf(IO_BUF, "reward given\n");
            return true;
        }
        else if(probability_roll > arm_probabilities[choice]){
    //        io_printf(IO_BUF, "no cigar\n");
            return false;
        }
        else{
    //        io_printf(IO_BUF, "shit broke\n");
        }
    }
}

// updates the current state of the pendulum
bool update_state(float time_step){
    float effective_force_pole_on_cart = 0;
    float pole_angle_force = (mass_pole * half_pole_length * pole_velocity * pole_velocity * sin(pole_angle));
    float angle_scalar = ((3.0f / 4.0f) * mass_pole * cos(pole_angle));
    float friction_and_gravity = (((friction_pole_hinge * pole_velocity) / (mass_pole * half_pole_length)) +
                        (gravity * sin(pole_angle)));
    float effective_pole_mass = mass_pole * (1.0f - ((3.0f / 4.0f) * cos(pole_angle) * cos(pole_angle)));

    effective_force_pole_on_cart = pole_angle_force + (angle_scalar * friction_and_gravity);
    if (cart_velocity > 0){
        cart_acceleration = (motor_force - friction_cart_on_track + effective_force_pole_on_cart) /
                                (mass_cart + effective_pole_mass);
    }
    else{
        cart_acceleration = (motor_force + friction_cart_on_track + effective_force_pole_on_cart) /
                                (mass_cart + effective_pole_mass);
    }

    float length_scalar = (-3.0f / 4.0f * half_pole_length);
    float cart_acceleration_effect = cart_acceleration * cos(pole_angle);
    float gravity_effect = gravity * sin(pole_angle);
    float friction_effect = (friction_pole_hinge * pole_velocity) / (mass_pole * half_pole_length);
    pole_acceleration = length_scalar * (cart_acceleration_effect + gravity_effect + friction_effect);

    cart_velocity = (cart_acceleration * time_step) + cart_velocity;
    cart_position = (cart_velocity * time_step) + cart_position;

    pole_velocity = (pole_acceleration * time_step) + pole_velocity;
    pole_position = (pole_velocity * time_step) + pole_position;

    if (cart_position)

    if ('in bounds'){
        return true;
    }
    else{
        return false;
    }
}

void mc_packet_received_callback(uint key, uint payload)
{
    uint32_t compare;
    compare = key & 0x7;
//    io_printf(IO_BUF, "compare = %x\n", compare);
//    io_printf(IO_BUF, "key = %x\n", key);
//    io_printf(IO_BUF, "payload = %x\n", payload);
    use(payload);
    if(compare == BACKWARD_MOTOR){
        motor_force = motor_force - force_increment;
        if (motor_force < min_motor_force){
            motor_force = min_motor_force;
        }
    }
    else if(compare == FORWARD_MOTOR){
        motor_force = motor_force + force_increment;
        if (motor_force > max_motor_force){
            motor_force = max_motor_force;
        }
    }
}

void timer_callback(uint unused, uint dummy)
{
    use(unused);
    use(dummy);

    _time++;
    score_change_count++;

    if (!infinite_run && _time >= simulation_ticks)
    {
        //spin1_pause();
        recording_finalise();
        // go into pause and resume state to avoid another tick
        simulation_handle_pause_resume(resume_callback);
        //    spin1_callback_off(MC_PACKET_RECEIVED);

        io_printf(IO_BUF, "infinite_run %d; time %d\n",infinite_run, _time);
        io_printf(IO_BUF, "simulation_ticks %d\n",simulation_ticks);
        //    io_printf(IO_BUF, "key count Left %u\n", left_key_count);
        //    io_printf(IO_BUF, "key count Right %u\n", right_key_count);

        io_printf(IO_BUF, "Exiting on timer.\n");
//        simulation_handle_pause_resume(NULL);
        simulation_ready_to_read();

        _time -= 1;
        return;
    }
    // Otherwise
    else
    {
        // Increment ticks in frame counter and if this has reached frame delay
        tick_in_frame++;
        if(tick_in_frame == reward_delay)
        {
            if (was_there_a_reward()){
                add_reward();
            }
            else{
                add_no_reward();
            }
            // Reset ticks in frame and update frame
            tick_in_frame = 0;
//            update_frame();
            // Update recorded score every 1s
            if(score_change_count>=1000){
                if(reward_based == 0){
                    recording_record(0, &correct_pulls, 4);
                }
                else{
                    recording_record(0, &current_score, 4);
                }
                score_change_count=0;
            }
        }
    }
//    io_printf(IO_BUF, "time %u\n", ticks);
//    io_printf(IO_BUF, "time %u\n", _time);
}
//-------------------------------------------------------------------------------

INT_HANDLER sark_int_han (void);


//-------------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Entry point
//----------------------------------------------------------------------------
void c_main(void)
{
  // Load DTCM data
  uint32_t timer_period;
  if (!initialize(&timer_period))
  {
    io_printf(IO_BUF,"Error in initialisation - exiting!\n");
    rt_error(RTE_SWERR);
    return;
  }

  tick_in_frame = 0;

  // Set timer tick (in microseconds)
  io_printf(IO_BUF, "setting timer tick callback for %d microseconds\n",
              timer_period);
  spin1_set_timer_tick(timer_period);

  io_printf(IO_BUF, "simulation_ticks %d\n",simulation_ticks);

  // Register callback
  spin1_callback_on(TIMER_TICK, timer_callback, 2);
  spin1_callback_on(MC_PACKET_RECEIVED, mc_packet_received_callback, -1);

  _time = UINT32_MAX;

  simulation_run();




}
