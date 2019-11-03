///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Aidan Curtis & Patrick Han
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// Planner includes
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"
#include <fstream>

#include <cmath>

const double GRAVITY = 9.81;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpacePtr space) : ProjectionEvaluator(space)
    // PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 1;
        // return 0;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        // const double *values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        // projection(0)
        // projection(0) = (values[0] + values[1]) / 2.0;
        // projection(1) = (values[2] + values[3]) / 2.0;
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * c,
                 ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics

    // Retreive control value of torque
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double t = u[0]; // Torque

    const double theta = q[0]; // Retrieve angle
    const double omega = q[1]; // Retrieve velocity

    qdot.resize(q.size(), 0); // Initialize qdot as zeros

    qdot[0] = omega;
    qdot[1] = -1 * GRAVITY * cos(theta) + t;
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.
    // return nullptr;

    // STATE SPACE SETUP
    ompl::base::StateSpacePtr so2r1;

    // Create R^1 component of the State Space (angular velocity omega)
    auto r1 = std::make_shared<ompl::base::RealVectorStateSpace>(1);

    // Set bounds on R^1
    ompl::base::RealVectorBounds bounds(1);
    bounds.setLow(-10);  // omega (rotational velocity) has a min of -10
    bounds.setHigh(10);  // omega (rotational velocity) has a max of 10

    // Set the bounds on R^1
    r1->setBounds(bounds);

    // Create S1/SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create compound state space (R^1 x SO(2))
    so2r1 =  so2 + r1;


    // CONTROL SPACE SETUP
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(so2r1, 1); // Take in our state space, with 1 control
    
    // Set bounds on our control space torque
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-1 * abs(torque));
    cbounds.setHigh(abs(torque));
    controlSpace->setBounds(cbounds); // Set control bounds on the control space

    // Define a simple setup class
    ompl::control::SimpleSetup ss(controlSpace);

    // Return simple setup ptr
    ompl::control::SimpleSetupPtr ssptr = std::make_shared<ompl::control::SimpleSetup>(ss); // have no idea if this is right lol, whats up with SimpleSetupPtr?
    // ompl::control::SimpleSetupPtr ssptr(ss);
    // ompl::control::SimpleSetupPtr ssptr;
    return ssptr;

}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *so2r1state = state->as<ompl::base::CompoundState>();
   
    // extract the second component of the state and cast it to what we expect
    const auto *thet = so2r1state->as<ompl::base::SO2StateSpace::StateType>(0);

    // extract the first component of the state and cast it to what we expect
    const auto *omeg = so2r1state->as<ompl::base::RealVectorStateSpace::StateType>(1);

    return si->satisfiesBounds(state);// && (const void*)omeg != (const void*)thet;
}

void planPendulum(ompl::control::SimpleSetupPtr & ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.

    //auto cspace = ss->getControlSpace();
    auto space = ss->getStateSpace();

    // construct an instance of  space information from this control space
    //auto si(std::make_shared<ompl::control::SpaceInformation>(space, cspace));

    ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ss->getSpaceInformation(), &pendulumODE));


    // set the state propagation routine
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    // ss->setStatePropagator(propagate);
    ss->getSpaceInformation()->setPropagationStepSize(0.01);
    // set state validity checking for this space
    ss->setStateValidityChecker([&ss](const ompl::base::State *state) { return isStateValid(ss->getSpaceInformation().get(), state); });
    



    // Create start state
    ompl::base::ScopedState<> start(space);
    start[0] = -1 * M_PI/2; // Initial position
    start[1] = 0; // Initial velocity

    // Create goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI/2; // Goal position
    goal[1] = 0; // Goal velocity

    // set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    // set planner depending on user input choice
    if (choice == 1) // RRT
    {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    }
    else if (choice == 2) // KPIECE1
    {
        ompl::base::PlannerPtr kplanner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());

        space->registerProjection("PendulumProjection", ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space)));
        kplanner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("PendulumProjection");

        ss->setPlanner(kplanner);
    }
    else if (choice == 3) // RG-RRT
    {
        // ss->setPlanner(std::make_shared<ompl::control::RG-RRT>(ss->getSpaceInformation()));
    }
    

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = ss->solve(100.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        

        std::ofstream fout("path.txt");
        // print the path to screen
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
    } 
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
