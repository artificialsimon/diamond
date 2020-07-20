//// Simulation
#include <ode_robots/simulation.h>
// Noise generator
#include <selforg/noisegenerator.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>
// Robot's controller
#include "../../controller/diamond.h"
// Robot's wiring
#include <selforg/one2onewiring.h>
// The robot
#include <ode_robots/fourwheeled.h> 
// used arena
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

#include <gsl/gsl_histogram.h>
#include <ode_robots/terrainground.h>

//#include "../../utils/logeable.h"
//#include <selforg/logeable.h>
#include <selforg/stl_adds.h>
#include <sys/stat.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace lpzrobots;
bool toLog;
double zsize;
//double epsA;
//double epsC;
int terrain;
string logFileName;
bool useExtendedModel;
double stuckness;
double sigma_sqr_LWR;
int layers;

class ThisSim : public Simulation
{
  public:
    int bin_x, bin_y, coverage, displacement;
    OdeRobot* robot;
    Diamond* controller;
    //Logeable* logo;
    double error;
    int cover[10][10];
    bool upside_down;
    Pos position;

    ThisSim() { }

    ~ThisSim() {
      stuckness = 100. * stuckness / globalData.sim_step;
      cout << "stuck percentage: " << stuckness << endl;
      cout << "log: " << toLog << endl;
      if (toLog) {
        bool writeHeader;
        FILE* pFile;
        string fileName = to_string(layers) + ".txt";
        string header = "coverage\tzsize\tdisplacement\tupdise_down\ttime_steps\tstuck_percentage";
        struct stat stFileInfo;
        if ((stat(fileName.c_str(), &stFileInfo) != 0) && (!header.empty())) 
          writeHeader = true;
        else 
          writeHeader = false;
        cout << "Log file name: " << fileName << "\n";
        pFile = fopen(fileName.c_str(), "a");
        string ss;
        ss = to_string(coverage)  + "\t" + to_string(zsize) 
            + "\t" + to_string(displacement) + "\t" + to_string(upside_down) 
            + "\t" + to_string(globalData.sim_step) 
            + "\t" + to_string(stuckness);
        if (writeHeader) {
          fprintf(pFile, "%s\n", header.c_str());
          writeHeader = false;
        }
        fprintf(pFile, "%s\n", ss.c_str());
        fclose(pFile);
        /*
        logo = new Logeable(logFileName,
            "coverage\terror\tzsize\tepsa\tepsc\tdisplacement\tupdise_down\ttime_steps\tstuck_percentage");
        string ss;
        ss = to_string(coverage)  + "\t" + to_string(error) + "\t" + to_string(zsize) 
            + "\t" + to_string(epsA) + "\t" + to_string(epsC) + "\t" 
            + to_string(displacement) + "\t" + to_string(upside_down) 
            + "\t" + to_string(globalData.sim_step) 
            + "\t" + to_string(stuckness);
        logo->toLog(ss);
        */
      }
    }
     
    /// start() is called at the start and should create all the object (obstacles, agents...).
    virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
      // Initial position and orientation of the camera (use 'p' in graphical window to find out)
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
      // Some simulation parameters can be set here
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8);

      /** New robot instance */
      // Get the default configuration of the robot
      FourWheeledConf robotConf = FourWheeled::getDefaultConf();
      robotConf.useBumper = false;
      robotConf.useButton = false;
      //robotConf.useBigBox = true;
      robotConf.force     = 10;
      // Instantiating the robot
      robot = new FourWheeled(odeHandle, osgHandle, robotConf, "Four wheel robot");
      // Placing the robot in the scene
      ((OdeRobot*)robot)->place(Pos(.0, .0, .1));
      // Instantiating the model and controller
      DiamondConf diamond_conf;
      diamond_conf = Diamond::getDefaultConf();
      diamond_conf.n_layers = layers;
      controller = new Diamond(diamond_conf);
      //controller->setParam("epsA",0.3); // model learning rate
      //controller->setParam("epsC",0.3); // controller learning rate
      //controller->setParam("rootE",3);    // model and contoller learn with square rooted error
      global.configs.push_back ( controller );
      //controller->setParam("epsC", epsC); // 0.3
      //controller->setParam("Logarithmic", 0);

      // Create the wiring with color noise
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));
      // Create Agent
      OdeAgent* agent = new OdeAgent(global);
      // Agent initialisation
      agent->init(controller, robot, wiring);
      // Adding the agent to the agents list
      global.agents.push_back(agent);
      global.configs.push_back(agent);

      // Playground
      if (zsize >= .0) {
        double widthground = 20.;
        double heightground = 1.5;
        Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(widthground, 0.208, heightground));
        playground->setColor(Color(1., 1., 1., .99));
        playground->setPosition(osg::Vec3(0, 0, .0));
        if (terrain == 0) {
          Substance substance;
          substance.toRubber(5);
          playground->setGroundSubstance(substance);
          global.obstacles.push_back(playground);
          double xboxes = 18;//19.0;
          double yboxes = 17;
          double boxdis = 3.;//.45;//1.6;
          for (double j = .0; j < xboxes; j++)
            for(double i = .0; i < yboxes; i++) {
              double xsize = 1.;//1.0;
              double ysize = 1.;//.25;
              PassiveBox* b =
                new PassiveBox(odeHandle,
                    osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
              b->setPosition(Pos( + boxdis*(i-(xboxes-1)/2.0), + boxdis*(j-(yboxes-1)/2.0), 0.01));
              global.obstacles.push_back(b);
            }
        }
        if (terrain == 1) {
          TerrainGround* terrainground =
            new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
                "../../utils/terrain.ppm", "../../utils/maze256c.ppm",
                20, 20, zsize, OSGHeightField::Red);
          terrainground->setPose(osg::Matrix::translate(.0, .0, .11));
          global.obstacles.push_back(terrainground);
        }
      }

      bin_x = 0;
      bin_y = 0;
      coverage = 0;
      displacement = 0;
      upside_down = false;
      stuckness = .0;
      for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
          cover[i][j] = 0;

}

    /* Functions not used in this tutorial but typically useful */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
      if (toLog) {
        Position robot_position = robot->getPosition();
        const int playground = 20;
        const int bins = 10;
        double rx = robot_position.x + (playground / 2);
        double _bin_x = floor(rx / (playground / bins));
        double ry = robot_position.y + (playground /2);
        double _bin_y = floor(ry / (playground / bins));

        if (((fmod(rx, playground / bins) < 0.2) && (bin_x != _bin_x))  ||
            ((fmod(ry, playground / bins) < 0.2) && (bin_y != _bin_y))) {
          bin_x = _bin_x;
          bin_y = _bin_y;
          if (cover[bin_x][bin_y] == 0)
            coverage++;
          cover[bin_x][bin_y]++;
          displacement++;
        }

      }

      // this should be called at the end of the simulation but controller
      // already dead at ~ThisSim()
      error = 0.0; //controller->getXiAverage();
      
      Pose pp = robot->getMainPrimitive()->getPose();
      if (pp(0,2) < -.8) {
        simulation_time_reached = true;
        upside_down = true;
      }

      Pos po = robot->getMainPrimitive()->getPosition();
      if ((abs(position.x() - po.x()) < 0.000001) &&
          (abs(position.y() - po.y()) < 0.000001) &&
          (abs(position.z() - po.z()) < 0.000001)) {
        stuckness++;
      }
      position = po;
    }

    virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
      return false;
    }

    virtual void bindingDescription(osg::ApplicationUsage & au) const {
    }
};

int main (int argc, char **argv)
{
  // New simulation
  ThisSim sim;
  // set Title of simulation
  sim.setTitle("DIAMOND");
  sim.setCaption("Main Variant");
  int index;

  toLog = false;
  logFileName = "";
  index = Base::contains(argv, argc, "-log");
  if (index) {
    if (argc > index) {
       logFileName = argv[index];
       toLog = true;
     }
  }

  /*
  // controller learning rate
  epsC = .005; // 0.3;
  index = Base::contains(argv, argc, "-epsc");
  if(index) 
    if(argc > index)
      epsC = atof(argv[index]);

  // model learning rate
  epsA = .005; // 0.05;
  index = Base::contains(argv, argc, "-epsa");
  if(index) 
    if(argc > index)
      epsA = atof(argv[index]);
  */

  // Negative values cancel playground
  zsize = -1.;
  index = Base::contains(argv, argc, "-playground");
  if(index) 
    if(argc > index)
      zsize = atof(argv[index]);

  terrain = 0; // boxes
  index = Base::contains(argv, argc, "-terrain");
  if(index) 
    if(argc > index)
      terrain = atoi(argv[index]);

  useExtendedModel = true;
  index = Base::contains(argv, argc, "-noextendedmodel");
  if(index) 
    useExtendedModel = false;

  layers = 1;
  index = Base::contains(argv, argc, "-layers");
  if(index) 
    if(argc > index)
      layers = atoi(argv[index]);
  // Simulation begins
  return sim.run(argc, argv) ? 0 : 1;
}
