//
//  ECElevatorSim.cpp
//  
//
//  Created by Yufeng Wu on 6/27/23.
//  Elevator simulation

#include <algorithm>
#include <vector>
#include "ECElevatorSim.h"
#include <limits>

using namespace std;

//ElevatorState methods

    int ElevatorState :: stage(ECElevatorSimRequest& req)
    {
        if ((req.IsFloorRequestDone() == true) && (req.IsServiced() == true))
        {
            return -1;
        }
        else if ((req.IsFloorRequestDone() == true))
        {
            return req.GetFloorDest();
        }
        else
        {
            return req.GetFloorSrc();
        }

    }
    
  EC_ELEVATOR_DIR ElevatorState:: DeterminDir(ECElevatorSimRequest& req)
  {
    
    if (stage(ele.GetRequest()[ele.GetIndex()]) > ele.GetCurrFloor())
    {
        return EC_ELEVATOR_UP;
    }
    else if (stage(ele.GetRequest()[ele.GetIndex()]) < ele.GetCurrFloor())
    {
        return EC_ELEVATOR_DOWN;
    } 
    else
    {
        return EC_ELEVATOR_STOPPED;
    }
  }
  
  EC_ELEVATOR_DIR ElevatorState :: DirOfReq(ECElevatorSimRequest& req)
  {
    if (ele.GetCurrFloor() < stage(req))
    {
        return EC_ELEVATOR_UP;
    }
    else if (ele.GetCurrFloor() > stage(req))
    {
        return EC_ELEVATOR_DOWN;
    }
    else
    {
        return EC_ELEVATOR_STOPPED;
    }
  }
  
  int ElevatorState :: closestreq(vector<int> list)
  {
    int min = abs(stage(ele.GetReq(list[0])) - ele.GetCurrFloor());
    int newI = 0;
    for (int j = 1; j <= list.size()-1; j++)
    {
       
        if (min > abs(stage(ele.GetReq(list[j])) - ele.GetCurrFloor()))
        {
            min = abs(stage(ele.GetReq(list[j])) - ele.GetCurrFloor());
            newI = j;
        }        
    }
    return list[newI];
   
  }

  void ElevatorState :: NextReq2()
  {
    vector <int> possible;
    vector <int> otherdir;
    for (int i = 0; i <= ele.GetRequest().size()-1; i++)
    {
        if (stage(ele.GetRequest()[i]) == -1)
        {
            continue;
        }
        else if((ele.GetRequest()[i].GetTime() <= ele.GetTime()))
        {
            if (ele.GetCurrDir() == DirOfReq(ele.GetRequest()[i]))
            {
                possible.push_back(i);
                continue;
            }
            otherdir.push_back(i);
        }
    }
    
    if (possible.size() == 0)
    {
        if (otherdir.size() == 0)
        {
            return;
        }
        ele.SetIndex(closestreq(otherdir));
        return;    
    }
    ele.SetIndex(closestreq(possible));
    return;
  }

  void ElevatorStopped :: WhatsNext()
  { 
    
    if (ele.GetReqTime(ele.GetIndex()) != ele.GetTime())
    {
        ele.SetIndex(ele.NextIndex());
        return;
    }
    else  
    {
        ele.SetCurrDir(DeterminDir(ele.GetRequest()[ele.GetIndex()]));
        ele.PickUp();      
        ele.SetState(1);
        return;
    }   
  }

  void ElevatorMoving :: changeF(int goingtoF)
  {
    if (goingtoF > ele.GetCurrFloor())
    {
        ele.SetCurrFloor(ele.GetCurrFloor()+1);    
    }
    else
    {
        ele.SetCurrFloor(ele.GetCurrFloor()-1);    
    }
  }

    void ElevatorMoving :: Check()
    {
        changeF(ele.GetReq(ele.GetIndex()).GetFloorSrc());
        if (ele.GetCurrFloor() == ele.GetReq(ele.GetIndex()).GetFloorSrc())
        {
            ele.DropOff();
            ele.PickUp();
            ele.SetState(2);
            return;
        }
        ele.SetState(1);
        return;   

    }
    void ElevatorMoving :: Check2()
    {
        ele.DropOff();
        ele.PickUp();
        NextReq2();
            if (ele.AllDone() == true)
        {
            ele.SetState(0);
            return;
        }
        if ((ele.GetIndex() == ele.GetRequest().size()-1) &&(ele.GetRequest()[ele.GetIndex()].IsServiced() == true))
        {
            ele.SetState(0);
            return;   
        }
        ele.SetState(2);
        return;   

    }
  void ElevatorMoving :: WhatsNext()
  {
    NextReq2();
    if (!ele.GetReq(ele.GetIndex()).IsFloorRequestDone())
    {   
        Check();
        return; 
    }
    else  
    {
        if (ele.GetCurrFloor() == ele.GetReq(ele.GetIndex()).GetFloorDest())
        {
            Check2();
            return;  
        }
        changeF(ele.GetReq(ele.GetIndex()).GetFloorDest());
        if (ele.GetCurrFloor() == ele.GetReq(ele.GetIndex()).GetFloorDest())
        {
            Check2();
            return;   
        }
        ele.SetState(1);
        return;
    }
  }
  
  void ElevatorPause :: WhatsNext()
  {
    
    NextReq2();
    if((ele.GetRequest()[ele.GetIndex()].IsServiced()) && (ele.GetInEle().size() == 0 ))
    {
        ele.SetIndex(ele.NextIndex());
        ele.SetState(0);
        return;
    } 
   
    ele.SetCurrDir(DeterminDir(ele.GetRequest()[ele.GetIndex()]));       
    ele.SetState(1);
    return;
   }

   //ElevatorSim methods

    ECElevatorSim::ECElevatorSim(int numFloors, std::vector<ECElevatorSimRequest> &listRequests): numOfFloors(numFloors), listOfRequests(listRequests)
    {}

    // free buffer
    ECElevatorSim::~ECElevatorSim()
    {}

    // Simulate by going through all requests up to certain period of time (as specified in lenSim)
    // starting from time 0. For example, if lenSim = 10, simulation stops at time 10 (i.e., time 0 to 9)
    // Caution: the list of requests contain all requests made at different time;
    // at a specific time of simulation, some events may be made in the future (which you shouldn't consider these future requests)
    void ECElevatorSim::Simulate(int lenSim)
    {
        int time;
        ElevatorStopped* nextS = new ElevatorStopped(*this);
        ElevatorMoving* nextM  = new ElevatorMoving(*this);
        ElevatorPause* nextP = new ElevatorPause(*this);
        for (int time = 0; time < lenSim; time++)
        {
            this->SetTime(time);
           
            //0 == stopped
            //1 == moving 
            //2 == pause
           switch(this->GetState()) 
           {
                case 0:
                    {
                        nextS->WhatsNext();
                        break;
                    }
                case 1:
                    {
                        nextM->WhatsNext();
                        break;
                    }
                case 2:
                    {
                        nextP->WhatsNext();
                        break;
                    }    
            }
        }
    }

    // The following methods are about querying/setting states of the elevator
    // which include (i) number of floors of the elevator, 
    // (ii) the current floor: which is the elevator at right now (at the time of this querying). Note: we don't model the tranisent states like when the elevator is between two floors
    // (iii) the direction of the elevator: up/down/not moving
    ECElevatorSimRequest& ECElevatorSim ::GetReq(int i)
    {
            return listOfRequests[i];
    }

    int ECElevatorSim :: NextIndex()
    {
        bool first = false;
        vector<int> tie;
        int min = 0;
        int index = 0;
        for(int i = 0; i < listOfRequests.size(); i++)
        {
            if(listOfRequests[i].GetTime() > GetTime())
            { 
                if (first == false)
                {
                    min = listOfRequests[i].GetTime() - GetTime();
                    first = true;
                    index = i;
                    continue;
                }
                if (min > listOfRequests[i].GetTime() - GetTime())
                {
                    min = listOfRequests[i].GetTime() - GetTime();
                    index = i;
                }
            }
        }
        for(int i = 0; i < listOfRequests.size(); i++)
        {
            
            if(listOfRequests[i].GetTime() >= GetTime())
            { 
                if (min == listOfRequests[i].GetTime() - GetTime())
                {
                    tie.push_back(i);
                }
            }
        }
        if(tie.size() == 1)
        {
            return index;
        }
        bool first2 = false;
        int winner = 0;
        int closest = 0;
        for(int j =0; j < tie.size(); j++)
        {
            if (listOfRequests[j].GetFloorSrc() < CurrFloor)
            {
                continue;
            }
            winner = tie[j];
        }
        return winner;
    }

    bool ECElevatorSim :: AllDone()
    {
        bool done = true;
        for (int i =0; i <= listOfRequests.size()-1; i++)
        {
            if (listOfRequests[i].IsServiced() == false)
            {
                done = false;
            }
        }
        return done;
    }

    void  ECElevatorSim :: PickUp()
    {
        vector<int> eleToAdd;
        for (int i =0; i < GetRequest().size(); i++)
        {
            if ((GetCurrFloor() == GetRequest()[i].GetFloorSrc()) && (GetRequest()[i].GetTime() <= GetTime()) && (GetRequest()[i].IsServiced()==false))
            {
                GetRequest()[i].SetFloorRequestDone(true);
                vector<int>::iterator position = find(InEle.begin(), InEle.end(), i);
                if (position != InEle.end()) 
                {
                    InEle.erase(position);
                }
                InEle.push_back(i);
            }
        }
        return;
    }

    void  ECElevatorSim :: DropOff()
    {
        vector<int> eleToRem;
        for (int i =0; i < InEle.size(); i++)
        {
            if (GetCurrFloor() == GetRequest()[InEle[i]].GetFloorDest())
            {
                GetRequest()[InEle[i]].SetServiced(true);
                GetRequest()[InEle[i]].SetArriveTime(GetTime());
                eleToRem.push_back(InEle[i]);
            }
        }
        for(int j = 0; j < eleToRem.size(); j++)
        {
            RemFromEle(eleToRem[j]);
        }
        return;
    }

    void ECElevatorSim:: RemFromEle(int ind)
    {
       vector<int>::iterator position = find(InEle.begin(), InEle.end(), ind);
        if (position != InEle.end()) 
        {
            InEle.erase(position);
        }
    }

    void ECElevatorSim :: AddToEle(int i)
    {
      InEle.push_back(i);
    }
   
    vector<ECElevatorSimRequest>& ECElevatorSim:: GetRequest()
    {
         return listOfRequests;
    }

    int ECElevatorSim :: GetReqTime(int req)
    {
        return listOfRequests[req].GetTime();
    }

    int ECElevatorSim:: GetFlSrc(int req)
    {
        return listOfRequests[req].GetFloorSrc();
    }

    vector<int>& ECElevatorSim :: GetInEle()
    {
        return InEle;
    }

    int& ECElevatorSim::GetCurrFloor()
    {
        return CurrFloor;
    }

    void ECElevatorSim :: SetState(int St)
    {
        state = St;
    }

    void ECElevatorSim :: SetTime(int T)
    {
        t = T;
    }
    
    void ECElevatorSim::SetIndex(int I)
    {
        index = I;
    }

    void ECElevatorSim::SetCurrFloor(int f)
    {
        CurrFloor = f;
    }
    
    void ECElevatorSim::SetCurrDir(EC_ELEVATOR_DIR newdir)
    {
        dir = newdir;
    }

