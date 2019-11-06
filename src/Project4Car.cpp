///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
// The collision checker produced in project 2
#include "CollisionChecking.h"
#include <fstream>
#include <cmath>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    // CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    CarProjection(const ompl::base::StateSpacePtr space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.25;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd>  projection ) const override
    {
        // TODO: Your projection for the car
        const ompl::base::CompoundState* compound_state = state->as<ompl::base::CompoundState>();
        const ompl::base::RealVectorStateSpace::StateType* r2;
        r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

        projection(0) = r2->values[0];
        projection(1) = r2->values[1];

    }
};

void carODE(const ompl::control::ODESolver::StateType &  q , const ompl::control::Control * control ,
            ompl::control::ODESolver::StateType &  qdot )
{

    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    double theta = q[2];
    double v = q[3];

    qdot.resize(q.size(), 0); // Initialize qdot as zeros
    qdot[0] = v*cos(theta);
    qdot[1] = v*sin(theta);
    qdot[2] = u[0];
    qdot[3] = u[1];



}

void makeStreet(std::vector<Rectangle> &  obstacles )
{
    Rectangle obstacle1;
    obstacle1.x = -1.0;
    obstacle1.y = 0.8;
    obstacle1.width = 2;
    obstacle1.height = 0.2;
    obstacles.push_back(obstacle1);

    Rectangle obstacle2;
    obstacle2.x = -1.0;
    obstacle2.y = -0.5;
    obstacle2.width = 1.2;
    obstacle2.height = 1.0;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0.5;
    obstacle3.y = -0.5;
    obstacle3.width = 0.5;
    obstacle3.height = 1.0;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = -1.0;
    obstacle4.y = -1.0;
    obstacle4.width = 2.0;
    obstacle4.height = 0.2;
    obstacles.push_back(obstacle4);
}

// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::control::SpaceInformation *si, const ompl::base::State *state, std::vector<Rectangle> &obstacles)
{
    // Cast the state to a real vector state
    auto compound_state = state->as<ompl::base::CompoundState>();

    const ompl::base::RealVectorStateSpace::StateType* r2;
    r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    const ompl::base::SO2StateSpace::StateType* so2;
    so2 = compound_state->as<ompl::base::SO2StateSpace::StateType>(1);

    const ompl::base::RealVectorStateSpace::StateType* r;
    r = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(2);

    // Extract x, y
    double x = r2->values[0];
    double y = r2->values[1];
    if(!si->satisfiesBounds(state)){
        return false;
    }
    return isValidPoint(x, y, obstacles);
}


ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &  obstacles )
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // STATE SPACE SETUP
    ompl::base::StateSpacePtr r2so2r1;

    // Create R^1 component of the State Space (angular velocity omega)
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // Set bounds on R^2
    ompl::base::RealVectorBounds r2_bounds(2);

    r2_bounds.setLow(-1.0);  // omega,vdot
    r2_bounds.setHigh(1.0);  // omega,vdot

    // Set the bounds on R^1
    r2->setBounds(r2_bounds);

    // Create S1/SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create R^1 component of the State Space (angular velocity omega)
    auto r1 = std::make_shared<ompl::base::RealVectorStateSpace>(1);

    // Set bounds on R^1
    ompl::base::RealVectorBounds r1_bounds(1);
    r1_bounds.setLow(-1);  // omega, vdot
    r1_bounds.setHigh(1);  // omega, vdot

    r1->setBounds(r1_bounds);

    // Create compound state space
    r2so2r1 =  r2+so2+r1;    

    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(r2so2r1, 2); // Take our state space plus two for control
    
    ompl::base::RealVectorBounds control_bounds(2);

    control_bounds.setLow(0, - M_PI / 2);
    control_bounds.setHigh(0,  M_PI / 2);
    control_bounds.setLow(1, -0.1);
    control_bounds.setHigh(1, 0.1);

    controlSpace->setBounds(control_bounds);

        // Define a simple setup class
    ompl::control::SimpleSetup ss(controlSpace);

    // Return simple setup ptr
    ompl::control::SimpleSetupPtr ssptr = std::make_shared<ompl::control::SimpleSetup>(ss); // have no idea if this is right lol, whats up with SimpleSetupPtr?
    // ompl::control::SimpleSetupPtr ssptr(ss);
    // ompl::control::SimpleSetupPtr ssptr;

    ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (ssptr->getSpaceInformation(), &carODE));
    
        // set state validity checking for this space
    ssptr->setStateValidityChecker([&ssptr, &obstacles](const ompl::base::State *state) { return isValidStatePoint(ssptr->getSpaceInformation().get(), state, obstacles); });

    // set the state propagation routine
    ssptr->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    // ss->setStatePropagator(propagate);
    ssptr->getSpaceInformation()->setPropagationStepSize(0.05);

    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    auto space  = ssptr->getStateSpace();


    // Create start state
    ompl::base::ScopedState<> start(space);
    start[0] = -0.5; // Initial x
    start[1] = -0.6; // Initial y
    start[2] = 0; // Initial th
    start[3] = 0; // Initial vel


    // Create goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 0.75;  // Initial x
    goal[1] = 0.6; // Initial y
    goal[2] = 0; // Initial th
    goal[3] = 0; // Initial vel


    // set the start and goal states
    ssptr->setStartAndGoalStates(start, goal, 0.2);
    
    return ssptr;
}






void planCar(ompl::control::SimpleSetupPtr & ss, int choice)
{    
    if(choice == 1){
        ompl::base::PlannerPtr planner(new ompl::control::RRT(ss->getSpaceInformation()));
        ss->setPlanner(planner);
    }
    else if(choice == 2){
        ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss->getSpaceInformation()));

        auto space = ss->getStateSpace();
        // ompl::base::StateSpace *space_normal_ptr = space.get();
        space->registerProjection("CarProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(space)));
        planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("CarProjection");
        ss->setPlanner(planner);
    }
    else if(choice == 3){
        ompl::base::PlannerPtr planner(new ompl::control::RGRRT(ss->getSpaceInformation()));
        ss->setPlanner(planner);
    }
    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = ss->solve(100.0);


    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        std::ofstream fout("car_path.txt");
        // print the path to screen
        ss->getSolutionPath().printAsMatrix(fout);
        fout.close();
    } 
    else
    {
        std::cout << "No solution found" << std::endl;
    }
    
}

void benchmarkCar(ompl::control::SimpleSetupPtr & ss )
{
    // TODO: Do some benchmarking for the pendulum
    double runtime_limit = 60.0;
    double memory_limit = 100000.0;  // set high because memory usage is not always estimated correctly
    int run_count = 20;
    std::string benchmark_name = std::string("car");

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(*ss, benchmark_name);

    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss->getSpaceInformation())));

    auto space = ss->getStateSpace();

    ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss->getSpaceInformation()));
    space->registerProjection("CarProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(space)));
    planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("CarProjection");
    b.addPlanner(planner); 
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss->getSpaceInformation())));


    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
