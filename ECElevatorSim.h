//
//  ECElevatorSim.h
//  
//
//  Created by Yufeng Wu on 6/27/23.
//  Elevator simulation

#ifndef ECElevatorSim_h
#define ECElevatorSim_h

#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <string>
using namespace std;

//*****************************************************************************
// DON'T CHANGE THIS CLASS
// 
// Elevator simulation request: 
// (i) time: when the request is made
// (ii) floorSrc: which floor the user is at at present
// (iii) floorDest floor: where the user wants to go; we assume floorDest != floorSrc
// 
// Note: a request is in three stages:
// (i) floor request: the passenger is waiting at floorSrc; once the elevator arrived 
// at the floor (and in the right direction), move to the next stage
// (ii) inside request: passenger now requests to go to a specific floor once inside the elevator
// (iii) Once the passenger arrives at the floor, this request is considered to be "serviced"
//
// two sspecial requests:
// (a) maintenance start: floorSrc=floorDest=-1; put elevator into maintenance 
// starting at the specified time; elevator starts at the current floor
// (b) maintenance end: floorSrc=floorDest=0; put elevator back to operation (from the current floor)
//are the destinations of those currently inside the elevator prioritized over future requests
class ECElevatorSimRequest
{
public:
    ECElevatorSimRequest(int timeIn, int floorSrcIn, int floorDestIn) : time(timeIn), floorSrc(floorSrcIn), floorDest(floorDestIn), fFloorReqDone(false), fServiced(false), timeArrive(-1) 
    {
    // cout<<"R time in"<<time<<"R floorSrcIn"<< floorSrc<<" R floorDestIn"<< floorDest<<endl;
    } 
    
    ECElevatorSimRequest(const ECElevatorSimRequest &rhs) : time(rhs.time), floorSrc(rhs.floorSrc), floorDest(rhs.floorDest), fFloorReqDone(rhs.fFloorReqDone), fServiced(rhs.fServiced), timeArrive(rhs.timeArrive) {}
    int GetTime() const {return time; }
    int GetFloorSrc() const { return floorSrc; }
    int GetFloorDest() const { return floorDest; }
    bool IsGoingUp() const { return floorDest >= floorSrc; }

    // Is this passenger in the elevator or not
    bool IsFloorRequestDone() const { return fFloorReqDone; }
    void SetFloorRequestDone(bool f) { fFloorReqDone = f; }

    // Is this event serviced (i.e., the passenger has arrived at the desstination)?
    bool IsServiced() const { return fServiced; }
    void SetServiced(bool f) { fServiced = f; }

    // Get the floor to service
    // If this is in stage (i): waiting at a floor, return that floor waiting at
    // If this is in stage (ii): inside an elevator, return the floor going to
    // Otherwise, return -1
    int GetRequestedFloor() const {
        if( IsServiced() )  {
            return -1;
        }
        else if( IsFloorRequestDone() )   {
            return GetFloorDest();
        }
        else {
            return GetFloorSrc();
        }
    }

    // Wait time: get/set. Note: you need to maintain the wait time yourself!
    int GetArriveTime() const { return timeArrive; }
    void SetArriveTime(int t) { timeArrive = t; }

    // Check if this is the special maintenance start request
    bool IsMaintenanceStart() const { return floorSrc==-1 && floorDest==-1; }
    bool IsMaintenanceEnd() const { return floorSrc==0 && floorDest==0; }

private:
    int time;           // time of request made
    int floorSrc;       // which floor the request is made
    int floorDest;      // which floor is going
    bool fFloorReqDone;   // is this passenger passing stage one (no longer waiting at the floor) or not
    bool fServiced;     // is this request serviced already?
    int timeArrive;     // when the user gets to the desitnation floor
};

//*****************************************************************************
// Elevator moving direction
//Elevator state class and each different elevator state will inheirent from it
typedef enum
{
    EC_ELEVATOR_STOPPED = 0,    // not moving
    EC_ELEVATOR_UP,             // moving up
    EC_ELEVATOR_DOWN            // moving down
} EC_ELEVATOR_DIR;

//*****************************************************************************
// Add your own classes here...
//decide what to do next
//loop through time and retrun what states to do next
//sim increaments time 
//at each floor evaluate what is the next thing to do.  
class ECElevatorSim;
class ElevatorState
{
    public:
        ElevatorState(ECElevatorSim& ele) : ele(ele){};

        virtual ~ElevatorState(){};

        //determines the next elevator state and handles requests 
        virtual void WhatsNext() = 0;

        //figures out direction of elevator based on the current request being processed
        virtual EC_ELEVATOR_DIR DeterminDir(ECElevatorSimRequest& req);

        //determines the direction the elevator needs to go to fulfil the inputed request
        virtual EC_ELEVATOR_DIR DirOfReq(ECElevatorSimRequest& req);

        //sets the current index being process to the closest request in the list
        virtual int closestreq (vector<int> list);

        //determines what stage in processing the request is in and returns the flr that the elevator needs to go to 
        //to finish the current stage of processing 
        virtual int stage(ECElevatorSimRequest& req);

        //determines the next request that needs to be processed
        virtual void NextReq2();
        
        // virtual void Check();
    
    private:
    ECElevatorSim &ele;
};
//matinence
class ElevatorStopped : public ElevatorState
{
    //different reasons for why elevator could be stopped
    public:
        ElevatorStopped(ECElevatorSim& ele) : ElevatorState(ele), ele(ele){};
        void WhatsNext();

    private:
    ECElevatorSim &ele;
};
class ElevatorMoving : public ElevatorState
{
    public:
        ElevatorMoving(ECElevatorSim& ele) :  ElevatorState(ele), ele(ele){};
        void WhatsNext();
        
        //changes floor; dropoff and pick up passengers; and sets correct state
        void Check();

        //dropoff and pick up passengers;set next request; and sets correct state 
        void Check2();

        //updates the current floor value when the elevator moves
        void changeF(int goingtoF);
  
    private:
        ECElevatorSim &ele;
};
class ElevatorPause : public ElevatorState
{
    public:
        ElevatorPause(ECElevatorSim& ele) :  ElevatorState(ele), ele(ele){};
        void WhatsNext();
            
    private:
        ECElevatorSim &ele;
};

//*****************************************************************************
// Simulation of elevator

class ECElevatorSim
{
public:
    // numFloors: number of floors serviced (floors numbers from 1 to numFloors)
    
    ECElevatorSim(int numFloors, std::vector<ECElevatorSimRequest> &listRequests);

    // free buffer
    ~ECElevatorSim();

    // Simulate by going through all requests up to certain period of time (as specified in lenSim)
    // starting from time 0. For example, if lenSim = 10, simulation stops at time 10 (i.e., time 0 to 9)
    // Caution: the list of requests contain all requests made at different time;
    // at a specific time of simulation, some events may be made in the future (which you shouldn't consider these future requests)
    void Simulate(int lenSim);

    //return the list of requests
    vector<ECElevatorSimRequest>& GetRequest();
    // The following methods are about querying/setting states of the elevator
    // which include (i) number of floors of the elevator, 
    // (ii) the current floor: which is the elevator at right now (at the time of this querying). Note: we don't model the tranisent states like when the elevator is between two floors
    // (iii) the direction of the elevator: up/down/not moving

    // Get num of floors
    int GetNumFloors() const { return numOfFloors; }
    
    //checks if all request have been serviced
    bool AllDone();

    //Get index of the request being processed
    int GetIndex() const{return index;}

    //Sets index of the request being processed
    void SetIndex(int I);

    //Returns the time of the request at the inputed index was made
    int GetReqTime(int req);

    //Get time
    int GetTime() const{return t;}

    //Sets time
    void SetTime(int T);

    //Returns the floor of the request at the inputed index was made
    int GetFlSrc(int req);

    //Get the state of the elevator
    int GetState() const{return state;}

    //Adds to InEle which keeps track of the people inside the elevator
    void AddToEle(int i);

    //Returns the list of people inside the elevator
    vector<int>& GetInEle();

    //Returns the request at the inputed index
    ECElevatorSimRequest& GetReq(int i);

    //Determines the index of the next non-active request
    int NextIndex();

    //Removes from InEle
    void RemFromEle(int ind);

    //Sets the state of the elevator
    void SetState(int St);

    //Dropoffs off all passengers that need to get off at the current floor
    void DropOff();

    //Pickup all passengers that need to be picked up on the current floor
    void PickUp();

    // Get current floor
    int& GetCurrFloor();

    // Set current floor
    void SetCurrFloor(int f);

    // Get current direction
    EC_ELEVATOR_DIR GetCurrDir() const{return dir;};

    // Set current direction
    void SetCurrDir(EC_ELEVATOR_DIR newdir);

private:
    // Your code here
    int numOfFloors = 0;
    int CurrFloor = 1;
    int t = 0;
    int index = 0; 
    int state = 0;
    EC_ELEVATOR_DIR dir = EC_ELEVATOR_STOPPED;
    vector<ECElevatorSimRequest> &listOfRequests;
    vector<int> InEle;
};


#endif /* ECElevatorSim_h */
