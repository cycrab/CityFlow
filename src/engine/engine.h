#ifndef CITYFLOW_ENGINE_H
#define CITYFLOW_ENGINE_H

#include "flow/flow.h"
#include "roadnet/roadnet.h"
#include "engine/archive.h"
#include "utility/barrier.h"

#include <mutex>
#include <thread>
#include <set>
#include <random>
#include <fstream>


namespace CityFlow {

    class Engine {
        friend class Archive;
    private:
        static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) {
            return a.second > b.second;
        }

        std::map<int, std::pair<Vehicle *, int>> vehiclePool;
        std::map<std::string, Vehicle *> vehicleMap;
        std::vector<std::set<Vehicle *>> threadVehiclePool;
        std::vector<std::vector<Road *>> threadRoadPool;
        std::vector<std::vector<Intersection *>> threadIntersectionPool;
        std::vector<std::vector<Drivable *>> threadDrivablePool;
        std::vector<Flow> flows;
        RoadNet roadnet;
        int threadNum;
        double interval;
        bool saveReplay;
        bool saveReplayInConfig; // saveReplay option in config json
        bool warnings;
        std::vector<std::pair<Vehicle *, double>> pushBuffer;
        std::vector<Vehicle *> laneChangeNotifyBuffer;
        std::set<Vehicle *> vehicleRemoveBuffer;
        rapidjson::Document jsonRoot;
        std::string stepLog;

        size_t step = 0;
        size_t activeVehicleCount = 0;
        int seed;
        std::mutex lock;
        Barrier startBarrier, endBarrier;
        std::vector<std::thread> threadPool;
        bool finished = false;
        std::string dir;
        std::ofstream logOut;

        bool rlTrafficLight;
        bool laneChange;
        int manuallyPushCnt = 0;

        int finishedVehicleCnt = 0;
        double cumulativeTravelTime = 0;

    private:
        void vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer);

        void planRoute();

        void getAction();

        void updateAction();

        void updateLocation();

        void updateLeaderAndGap();

        void planLaneChange();


        void threadController(std::set<Vehicle *> &vehicles, 
                              std::vector<Road *> &roads,
                              std::vector<Intersection *> &intersections,
                              std::vector<Drivable *> &drivables);

        void threadPlanRoute(const std::vector<Road *> &roads);

        void threadGetAction(std::set<Vehicle *> &vehicles);

        void threadUpdateAction(std::set<Vehicle *> &vehicles);

        void threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables);

        void threadUpdateLocation(const std::vector<Drivable *> &drivables);

        void threadNotifyCross(const std::vector<Intersection *> &intersections);

        void threadInitSegments(const std::vector<Road *> &roads);

        void threadPlanLaneChange(const std::set<Vehicle *> &vehicles);

        void handleWaiting();

        void updateLog();

        bool checkWarning();

        bool loadRoadNet(const std::string &jsonFile);

        bool loadFlow(const std::string &jsonFilename);

        std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting=false) const;

        void scheduleLaneChange();

        void insertShadow(Vehicle *vehicle);

    public:
        std::mt19937 rnd;

        Engine(const std::string &configFile, int threadNum);
//在engine中写接口函数以对bus各种属性的设定
//在头文件engine.h中加入函数的声明
 std::map<std::string, int> getVehiclePassenger() const;//获取所有公交车的人数
 void setVehiclePassenger(const std::string &id, int passenger);//设置某公交车的上下车人数
 std::map<std::string, bool> getVehicletype() const;//获取所有车辆是否为公交车的信息
 void setVehicletype(const std::string &id, bool isbus);//设置某车为公交车
 std::map<std::string, std::string>  getBusstation(const std::string &id) const;//获取某公交车的公交车站信息
 void addBusstation(const std::string &id, std::string road,double position);//添加某公交车的一个公交车站信息
 void deleteBusstation(const std::string &id);//删除某公交车最近添加的一个公交车站

        double getInterval() const { return interval; }

        bool hasLaneChange() const { return laneChange; }

        bool loadConfig(const std::string &configFile);

        void notifyCross();

        void nextStep();

        bool checkPriority(int priority);

        void pushVehicle(Vehicle *const vehicle, bool pushToDrivable = true);

        void setLogFile(const std::string &jsonFile, const std::string &logFile);

        void initSegments();

        ~Engine();

        // RL related api

        void pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads);

        size_t getVehicleCount() const;

        std::vector<std::string> getVehicles(bool includeWaiting = false) const;

        std::map<std::string, int> getLaneVehicleCount() const;

        std::map<std::string, int> getLaneWaitingVehicleCount() const;

        std::map<std::string, std::vector<std::string>> getLaneVehicles();

        std::map<std::string, double> getVehicleSpeed() const;

        std::map<std::string, double> getVehicleDistance() const;

        std::string getLeader(const std::string &vehicleId) const;

        double getCurrentTime() const;

        double getAverageTravelTime() const;

        void setTrafficLightPhase(const std::string &id, int phaseIndex);

        void setReplayLogFile(const std::string &logFile);

        void setSaveReplay(bool open);

        void setVehicleSpeed(const std::string &id, double speed);

        void setRandomSeed(int seed) { rnd.seed(seed); }
        
        void reset(bool resetRnd = false);

        // archive
        void load(const Archive &archive) { archive.resume(*this); }
        Archive snapshot() { return Archive(*this); }
        void loadFromFile(const char *fileName);

        bool setRoute(const std::string &vehicle_id, const std::vector<std::string> &anchor_id);

        std::map<std::string, std::string> getVehicleInfo(const std::string &id) const;
    };

}

#endif //CITYFLOW_ENGINE_H
