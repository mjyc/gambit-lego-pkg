#ifndef __armlib_arm_h_
#define __armlib_arm_h_

#include <ros/ros.h>

#include <armlib/trajectory.h>

#include <pthread.h>
#include <vector>
#include <list>

/**
 * @brief Namespace containing the arm control library.
 */
namespace armlib {

/**
 * @brief Constants used for influencing the direction of rotation for continuous joints
 */
typedef enum {
    R_SHORTEST       /** Prefer shortest possible rotation */,
    R_NEGATIVE       /** Prefer clockwise motion */,
    R_POSITIVE       /** Prefer counter/anticlockwise motion */
} motion_direction;

/**
 * @brief Type containing a vector of preferred motion directions for continuous joints
 */
typedef std::vector<motion_direction> dir_vect;

/**
 * @brief High-level control interface for a generic robotic arm.
 *
 * The Arm class provides high-level control of a robotic arm.  It provides two primary modes
 * for control: a trajectory-based mode, in which a trajectory is pre-computed before
 * execution begins and is played back on the arm, and a real-time mode intended for closed-
 * loop control, in which arm motion is specified one point at a time, allowing motion to be
 * computed on-the-fly, such as from sensor data.
 *
 * This functionality is implemented as a library to provide very tightly coupled control.
 * Even in trajectory-based modes, trajectory status is always available and trajectories can
 * be quickly canceled.
 * 
 * This class must be extended by an implementation that provides the actual interface to
 * particular hardware.  If there are additional functions provided by specific hardware,
 * those may be exposed in child classes.  However, this abstract class should provide the 
 * basic functions necessary for arm movement.
 *
 * An instance of Arm may only be initialized in a valid, running ROS node context.
 */
class Arm {
    public:

    Arm();
    ~Arm();

    /**
     * @brief Sets velocity and acceleration limits of the arm.
     *
     * Sets the velocity and acceleration limits of the arm (at least in terms of software
     * control; certain arm hardware, such as the Barrett WAM, may provide separate hardware-
     * based limits.)
     *
     * @param vlimit the default maximum velocity to use when computing trajectories; this
     *  will be the velocity of the constant-velocity segment, provided that it can be
     *  reached given the acceleration settings in effect.
     * @param alimit the default acceleration at the beginning and end of trajectories.
     * @param hard_vlimit a safety velocity limit; each time an update step is going to be
     *  sent to the arm, it is checked against this limit.  If the limit is exceeded, an
     *  error message will be printed and the arm will not be moved.
     */
    void set_limits(float vlimit, float alimit, float hard_vlimit);

    /**
     * @brief Queries the arm for the actual encoder position.
     *
     * @return a vector of joint angles from arm encoder values
     */
    void get_actual_joint_pos(js_vect &position);

    /**
     * @brief Stops all arm motion.
     *
     * Immediately cancels all arm motion.  Any running trajectory is stopped, the trajectory
     * queue is flushed, and real-time control mode is canceled.  Can be used to verify that
     * the arm is not moving and that the trajectory queue is empty, or as a software 
     * emergency stop.
     */
    void stop_motion();

    /**
     * @brief Sets closed-loop or trajectory mode.
     *
     * Sets whether the arm is in real-time/closed-loop (step-at-a-time) mode, or in
     * trajectory mode.  While the arm is in closed-loop mode, it is moved by calling
     * move_toward periodically to give fine-grained targets to the arm.  When switching to
     * closed-loop mode, any running trajectory is halted and the trajectory queue is
     * purged; sending trajectories to the arm in closed-loop mode is prohibited.
     *
     * In trajectory mode, the arm is moved by go_to, play_trajectory, and similar
     * trajectory-based calls.  While in trajectory mode, using move_toward to send targets
     * is prohibited.
     *
     * @param mode true for closed-loop mode, false for trajectory mode
     */
    void set_closed_loop_mode(bool mode);

    /**
     * @brief Moves the arm in closed-loop mode.
     *
     * Commands the arm to take a single step closer to a target.  If the step would cause
     * the arm to execute a motion with a velocity lower than the specified velocity limit,
     * then the step is executed as-is.  If the motion would exceed the given velocity limit,
     * then the largest step toward the target that can be taken without exceeding the limit
     * is taken.
     *
     * This only has an effect when the arm is in closed-loop mode.  If the arm is not in
     * closed-loop mode, no movement is made.
     *
     * @param target a vector of joint angles for the arm to move to/toward
     * @param vlimit the maximum velocity at which the arm will move.  If not specified, the
     *  arm will move at the default maximum velocity.
     */
    void move_toward(js_vect &target, float vlimit=-1);

    /**
     * @brief Executes a trajectory to a target.
     *
     * Commands the arm to execute a trajectory to the given target.  If the arm is not
     * currently moving, the trajectory is computed from the current position to the target.
     * If the arm is moving, then the trajectory is computed from the end position of the
     * current last trajectory in the queue.
     *
     * Allows specification of the direction of rotation for continuous rotation joints.
     * R_SHORTEST specifies that the direction of rotation that will result in the shortest
     * trajectory should be used.  R_CW specifies only clockwise rotation, and R_CCW
     * specifies only counterclockwise rotation.  R_CW and R_CCW will be overridden to
     * R_SHORTEST if the shortest rotation would be very small (preventing near-360 degree
     * motion.)
     *
     * This only has an effect when the arm is not in closed-loop mode.  If the arm is in
     * closed-loop mode, no movement is made and no trajectories are added to the queue.
     *
     * go_to is asynchronous; when it is called the trajectory will be added to the queue
     * and it will immediately return.  If you want your program to wait until the trajectory
     * is finished before continuing, call the wait_until_stopped method after calling go_to,
     * or use go_to_sync instead.
     *
     * @param target a vector of joint angles for the arm to move to
     * @param preferred_directions the preffered direction of rotation for continuous
     *  rotation joints.  Has no effect on non-continuous joints.  If the vector is too
     *  short to specify for all joints, unspecified joints are assumed to be R_SHORTEST.
     * @param vlimit the maximum velocity that the trajectory will be allowed to reach. If
     *  not specified, the default maximum velocity will be used.
     * @param alimit the rate at which the arm should accelerate.  If not specified, the
     *  default acceleration will be used.
     */
    void go_to(js_vect &target, dir_vect &preferred_directions, float vlimit=-1, 
        float alimit=-1);

    /**
     * @brief Executes a trajectory to a target.
     *
     * Commands the arm to execute a trajectory to the given target.  If the arm is not
     * currently moving, the trajectory is computed from the current position to the target.
     * If the arm is moving, then the trajectory is computed from the end position of the
     * current last trajectory in the queue.
     *
     * This only has an effect when the arm is not in closed-loop mode.  If the arm is in
     * closed-loop mode, no movement is made and no trajectories are added to the queue.
     *
     * go_to is asynchronous; when it is called the trajectory will be added to the queue
     * and it will immediately return.  If you want your program to wait until the trajectory
     * is finished before continuing, call the wait_until_stopped method after calling go_to,
     * or use go_to_sync instead.
     *
     * @param target a vector of joint angles for the arm to move to
     * @param vlimit the maximum velocity that the trajectory will be allowed to reach. If
     *  not specified, the default maximum velocity will be used.
     * @param alimit the rate at which the arm should accelerate.  If not specified, the
     *  default acceleration will be used.
     */
    void go_to(js_vect &target, float vlimit=-1, float alimit=-1);

    /**
     * @brief Executes a trajectory to a target, blocking until completion.
     *
     * Synchronous version of go_to.  Commands the arm to execute a trajectory to the given
     * target and blocks program execution until the trajectory is complete.  See the
     * documentation for go_to for more details.
     *
     * This is a convenience function; if you need more control (such as specifying
     * directions of rotation) use a call to go_to() followed by wait_until_stopped() for
     * the same behavior.
     *
     * @param target a vector of joint angles for the arm to move to
     * @param vlimit the maximum velocity that the trajectory will be allowed to reach. If
     *  not specified, the default maximum velocity will be used.
     * @param alimit the rate at which the arm should accelerate.  If not specified, the
     *  default acceleration will be used.
     */
    void go_to_sync(js_vect &target, float vlimit=-1, float alimit=-1);

    void play_trajectory(const char *filename);

    /**
     * @brief Blocks until trajectory execution is complete.
     *
     * Blocks program execution until the arm has finished executing all of the trajectories
     * in the trajectory queue.  Should have no effect if the arm is in closed-loop mode.
     *
     * @param include_latency if true, waits for the period of the control latency of the
     *  arm drivers, after the trajectory queue drains, before returning, ensuring that the
     *  arm motion has stopped.  Note that the wait will always happen if true, regardless
     *  of whether or not there is actually a trajectory running when called.
     */
    void wait_until_stopped(bool include_latency=false);
    
    /**
     * @brief Checks for compliance with joint limits.
     *
     * @param position a vector of joint angles to check
     * @return true if the joint values are within limits, false otherwise
     */
    virtual bool check_joint_limits(js_vect &position) = 0;

    /**
     * @brief Checks for compliance with joint limits for arm DOFs only.
     *
     * @param position a vector of joint angles to check
     * @return true if the joint values are within limits, false otherwise
     */
    virtual bool check_arm_joint_limits(js_vect &position) = 0;
    
    /**
     * @brief Checks for compliance with joint limits for manipulator DOFs only.
     *
     * @param position a vector of joint angles to check
     * @return true if the joint values are within limits, false otherwise
     */
    virtual bool check_manip_joint_limits(js_vect &position) = 0;

    inline unsigned int get_n_dofs() { return _n_dofs;}
    inline unsigned int get_n_arm_dofs() { return _n_arm_dofs; }
    inline unsigned int get_n_manip_dofs() { return _n_manip_dofs; }

    /**
     * @brief Computes inverse kinematics for translation and rotation matrix.
     *
     * @param position a 3-element array for the translation matrix
     * @param orientation a 9-element (3x3, row-major) rotation matrix
     * @param solutions a vector of js_vects to be populated with the solutions found
     * @return true if at least one solution was found; false if no valid solutions
     */
    virtual bool inverse_kinematics(double *position, double *orientation,
        std::vector<js_vect> &solutions) = 0;

    virtual bool inverse_kinematics(double x, double y, double z, double roll, double pitch,
        double yaw, std::vector<js_vect> &solutions);

    virtual js_vect closest_point(std::vector<js_vect> &candidates, js_vect &reference,
        dir_vect &preferred_directions);

    virtual js_vect closest_point(std::vector<js_vect> &candidates, js_vect &reference);

    protected:

    static const float SHORTEST_ANGLE_TOLERANCE = 0.01;
    
    /*
     * Stops the control thread; generally it is not necessary to call this, as it is done
     * automatically by the destructor.
     */
    void stop();
    
    static const float update_period = 0.05;    /** How often the control loop runs */

    /**
     * @brief Locks mutex for thread synchronization
     */
    inline void lock() { pthread_mutex_lock(&_lock); }

    /**
     * @brief Unlocks mutex for thread synchronization
     */
    inline void unlock() { pthread_mutex_unlock(&_lock); }
    
    /**
     * @brief Control loop.
     *
     * This is the main control loop that runs asynchronously as a separate thread and sends
     * motion targets to the arm.  It is implemented as a static method that takes a pointer
     * to an Arm object as its parameter, so that it can be the entry point for pthreads.
     */
    static void *control_thread_loop(void *obj);

    /**
     * @brief Low-level, hardware-specific call to get actual arm position.
     *
     * To be implemented by implementations of the Arm class.  This function should implement
     * the reading of encoder values (or other method of getting joint positions from the
     * arm.)
     *
     * @param position a vector that will be set to the actual joint positions, in radians
     */
    virtual bool get_encoder_pos(js_vect &position) = 0;

    /**
     * @brief Low-level, hardware-specific call to set joint targets.
     *
     * To be implemented by implementations of the Arm class.  This function should implement
     * the sending of new target joint angles to the arm.
     *
     * @param position a vector of joint angles in radians that will be sent to the arm
     */
    virtual bool set_target_pos(js_vect &position) = 0;


    unsigned int _n_dofs;               /** Number of DOFs in this arm */
    unsigned int _n_arm_dofs;           /** Number of DOFs belonging to arm */
    unsigned int _n_manip_dofs;         /** Number of DOFs belonging to manipulator */
    float _max_velocity;                /** Default velocity limit */
    float _max_accel;                   /** Default acceleration */
    float _hard_limit;                  /** Velocity that should never be exceeded */
    js_vect _joint_weights;             /** Weights of DOFs in velocity computation */
    unsigned int _latency_usec;         /** Smoothing latency of arm driver */
    std::vector<bool> _joints_cr;       /** Whether joints are continuous rotation */

    bool _should_terminate;             /** Set to true when thread should exit */
    pthread_mutex_t _lock;              /** Lock for data structures */
    pthread_t _control_thread;          /** Control thread */
    ros::Rate _control_loop_rate;       /** Rate in Hz for control thread to run */

    bool _closed_loop_mode;             /** Whether the arm is in closed-loop mode */
    bool _at_target;                    /** Whether the arm has reached its target */
    double _traj_start_time;            /** Time at which trajectory started */
    js_vect _commanded_pos;             /** Commanded position of the arm */
    Trajectory *_cur_traj;              /** Pointer to currently executing trajectory */
    std::list<Trajectory*> _traj_queue; /** Queue of trajectories waiting to execute */
};

}

#endif // __armlib_arm_h_
