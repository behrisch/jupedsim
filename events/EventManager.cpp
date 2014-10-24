#include "EventManager.h"

using namespace std;

/*******************
 Konstruktoren
 ******************/

#define UPDATE_FREQUENCY 1 // in seconds


EventManager::EventManager(Building *_b){
    _event_times=vector<double>();
    _event_types=vector<string>();
    _event_states=vector<string>();
    _event_ids=vector<int>();
    _projectFilename = "";
    _building = _b;
    _eventCounter=0;
    _dynamic=false;
    _file = fopen("../events/events.txt","r");
    _lastUpdateTime=0;
    _deltaT=0;
    if(!_file){
        Log->Write("INFO:\tDatei events.txt nicht gefunden. Dynamisches Eventhandling nicht moeglich.");
    }
    else{
        Log->Write("INFO:\tDatei events.txt gefunden. Dynamisches Eventhandling moeglich.");
        _dynamic=true;
    }
}

/*******************
 Dateien einlesen
 ******************/
void EventManager::SetProjectFilename(const std::string &filename){
    _projectFilename=filename;
}

void EventManager::SetProjectRootDir(const std::string &filename){
    _projectRootDir= filename;
}

void EventManager::readEventsXml(){
     Log->Write("INFO: \tLooking for events");
     //get the geometry filename from the project file
     TiXmlDocument doc(_projectFilename);
     if (!doc.LoadFile()){
          Log->Write("ERROR: \t%s", doc.ErrorDesc());
          Log->Write("ERROR: \t could not parse the project file.");
          exit(EXIT_FAILURE);
     }

     TiXmlElement* xMainNode = doc.RootElement();
     string eventfile="";
     if(xMainNode->FirstChild("events")){
          eventfile=_projectRootDir+xMainNode->FirstChild("events")->FirstChild()->Value();
          Log->Write("INFO: \tevents <"+eventfile+">");
     }
     else
     {
          Log->Write("INFO: \tNo events found");
          return;
     }

     Log->Write("INFO: \tParsing the event file");
     TiXmlDocument docEvent(eventfile);
     if(!docEvent.LoadFile()){
          Log->Write("ERROR: \t%s",docEvent.ErrorDesc());
          Log->Write("ERROR: \t could not parse the event file.");
          return;
     }

     TiXmlElement* xRootNode = docEvent.RootElement();
     if(!xRootNode){
          Log->Write("ERROR:\tRoot element does not exist.");
          exit(EXIT_FAILURE);
     }

     if( xRootNode->ValueStr () != "JPScore" ) {
          Log->Write("ERROR:\tRoot element value is not 'JPScore'.");
          exit(EXIT_FAILURE);
     }

     TiXmlNode* xEvents = xRootNode->FirstChild("events");
     if(!xEvents){
          Log->Write("ERROR:\tNo events found.");
          exit(EXIT_FAILURE);
     }

     for(TiXmlElement* e = xEvents->FirstChildElement("event"); e; e= e->NextSiblingElement("event")){
          _event_times.push_back(atoi(e->Attribute("time")));
          _event_types.push_back(e->Attribute("type"));
          _event_states.push_back(e->Attribute("state"));
          _event_ids.push_back(atoi(e->Attribute("id")));
     }
     Log->Write("INFO: \tEvents were read\n");
}

void EventManager::listEvents(){
    if(_event_times.size()==0){
        Log->Write("INFO: \tNo events in the events.xml");
    }
    else{
        char buf[10],buf2[10];
        for(unsigned int i=0;i<_event_times.size();i++){
            sprintf(buf,"%f",_event_times[i]);
            sprintf(buf2,"%d",_event_ids[i]);
            Log->Write("INFO: \tAfter "+string(buf)+" sec: "+_event_types[i]+" "+string(buf2)+" "+_event_states[i]);
        }
    }

}

void EventManager::readEventsTxt(double time){
    rewind(_file);
    char cstring[256];
    int lines=0;
    do{
        fgets(cstring,30,_file);
        if(cstring[0]!='#'){// keine Kommentarzeile
            lines++;
            if(lines>_eventCounter){
                Log->Write("INFO:\tEvent: after %f sec: ",time);
                getTheEvent(cstring);
                _eventCounter++;
            }
        }
     }while (feof(_file)==0);
}

/***********
 Update
 **********/

void EventManager::Update_Events(double time, double d){
     //1. pruefen ob in _event_times der zeitstempel time zu finden ist. Wenn ja zu 2. sonst zu 3.
     //2. Event aus _event_times und _event_values verarbeiten (Tuere schliessen/oeffnen, neues Routing)
     //   Dann pruefen, ob eine neue Zeile in der .txt Datei steht
     //3. .txt Datei auf neue Zeilen pruefen. Wenn es neue gibt diese Events verarbeiten ( Tuere schliessen/oeffnen,
     //   neues Routing) ansonsten fertig

     _deltaT=d;
     vector<Pedestrian*> _allPeds=_building->GetAllPedestrians();

     //zuerst muss geprueft werden, ob die Peds, die die neuen Infos schon haben sie an andere Peds weiter-
     //leiten muessen (wenn diese sich in der naechsten Umgebung befinden)
     int currentTime = _allPeds[0]->GetGlobalTime();
     if(currentTime!=_lastUpdateTime)
          if((currentTime%UPDATE_FREQUENCY)==0) {

               for(unsigned int p1=0;p1<_allPeds.size();p1++)
               {
                    Pedestrian* ped1 = _allPeds[p1];
                    if(ped1->GetNewEventFlag()){
                         int rID = ped1->GetRoomID();
                         int srID = ped1->GetSubRoomID();

                         for(unsigned int p2=0;p2<_allPeds.size();p2++)
                         {
                              Pedestrian* ped2 = _allPeds[p2];
                              //same room and subroom
                              if(rID==ped2->GetRoomID() && srID==ped2->GetSubRoomID())
                              {
                                   if(!ped2->GetNewEventFlag()&&ped2->GetReroutingTime()>2.0){
                                        //wenn der Pedestrian die neuen Infos noch nicht hat und eine Reroutingtime von > 2 Sekunden hat, pruefen ob er nah genug ist
                                        Point pos1 = ped1->GetPos();
                                        Point pos2 = ped2->GetPos();
                                        double distX = pos1.GetX()-pos2.GetX();
                                        double distY = pos1.GetY()-pos2.GetY();
                                        double dist = sqrt(distX*distX+distY*distY);
                                        if(dist<=J_EPS_INFO_DIST){// wenn er nah genug (weniger als 2m) ist, Info weitergeben (Reroutetime auf 2 Sek)
                                             //ped->RerouteIn(2.0);
                                             ped2->RerouteIn(0.0);
                                        }
                                   }
                              }
                         }
                    }
               }
               _lastUpdateTime=currentTime;
               //cout<<"updating..."<<currentTime<<endl<<endl;
          }

     //dann muss die Reroutingzeit der Peds, die die neuen Infos noch nicht haben, aktualisiert werden:
     for(unsigned int p1=0;p1<_allPeds.size();p1++)
     {
          Pedestrian* ped1 = _allPeds[p1];
          ped1->UpdateReroutingTime();
          if(ped1->IsReadyForRerouting()){
               ped1->ClearMentalMap();
               ped1->ResetRerouting();
               ped1->SetNewEventFlag(true);
          }
     }

     //Events finden
     for(unsigned int i=0;i<_event_times.size();i++){
          if(fabs(_event_times[i]-time)<J_EPS_EVENT){
               //Event findet statt
               Log->Write("INFO:\tEvent: after %f sec: ",time);
               if(_event_states[i].compare("close")==0){
                    closeDoor(_event_ids[i]);
               }
               else{
                    openDoor(_event_ids[i]);
               }
          }
     }
     if(_dynamic)
          readEventsTxt(time);
}

/***************
 Eventhandling
 **************/
void EventManager::closeDoor(int id){
    //pruefen ob entsprechende Tuer schon zu ist, wenn nicht dann schliessen und neues Routing berechnen
    Transition *t=_building->GetTransition(id);
    if(t->IsOpen()){
        t->Close();
        Log->Write("\tDoor %d closed.",id);
        changeRouting(id,"close");
    }
    else{
        Log->Write("Door %d is already close yet.", id);
    }

}

void EventManager::openDoor(int id){
    //pruefen ob entsprechende Tuer schon offen ist, wenn nicht dann oeffnen und neues Routing berechnen
    Transition *t=_building->GetTransition(id);
    if(!t->IsOpen()){
        t->Open();
        Log->Write("\tDoor %d opened.",id);
        changeRouting(id,"open");
    }
    else{
        Log->Write("Door %d is already open yet.", id);
    }
}

void EventManager::changeRouting(int id, string state){
    RoutingEngine* routingEngine= _building->GetRoutingEngine();
    routingEngine->Init(_building);
    _building->InitPhiAllPeds(_deltaT);
    vector<Pedestrian*> _allPedestrians=_building->GetAllPedestrians();
    unsigned int nSize = _allPedestrians.size();

    //Pedestrians sollen, damit es realitaetsnaeher wird, je nachdem wo sie stehen erst spaeter(abh. von der
    //Entfernung zur Tuer) merken, dass sich Tueren aendern. Oder sie bekommen die Info von anderen Pedestrians
    Transition *t = _building->GetTransition(id);
    //Abstand der aktuellen Position des Pedestrians zur entsprechenden Tuer: Tuer als Linie sehen und mit
    //DistTo(ped.GetPos()) den Abstand messen. Reroutezeit dann aus Entfernung und Geschwindigkeit berechnen.
    Line* l = new Line(t->GetPoint1(),t->GetPoint2());
    for (unsigned int p = 0; p < nSize; ++p) {
        //if(_allPedestrians[p]->GetExitIndex()==t->GetUniqueID()){
        _allPedestrians[p]->SetNewEventFlag(false);
        double dist = l->DistTo(_allPedestrians[p]->GetPos());
        Point v = _allPedestrians[p]->GetV();
        double norm =sqrt((v.GetX()*v.GetX())+(v.GetY()*v.GetY()));
        if(norm==0.0){
            norm=0.01;
        }
        double time = dist/norm;
        if(time<1.0){
            _allPedestrians[p]->ClearMentalMap();
            _allPedestrians[p]->ResetRerouting();
            _allPedestrians[p]->SetNewEventFlag(true);
        }
        else{
            _allPedestrians[p]->RerouteIn(time);
        }
        //if(dist>0.0&&dist<0.5){
          // _allPedestrians[p]->ClearMentalMap();
        //}
        //else if(dist>=0.5&&dist<3.0){
          // _allPedestrians[p]->RerouteIn(1.0);
        //}
        //else{
          // _allPedestrians[p]->RerouteIn(2.0);
        //}
        //}
        //else{
          //  _allPedestrians[p]->ClearMentalMap();
        //}
    }
}

void EventManager::getTheEvent(char* c){
    int split = 0;
    string type = "";
    string id = "";
    string state = "";
    for(int i=0;i<20;i++){
        if(!c[i]){
            break;
        }
        else if(c[i]==' '){
            split++;
        }
        else if(c[i]=='\n'){

        }
        else{
            if(split==0){
                type+=c[i];
            }
            else if(split==1){
                id+=c[i];
            }
            else if(split==2){
                state+=c[i];
            }
        }

    }
    if(state.compare("close")==0){
        closeDoor(atoi(id.c_str()));
    }
    else{
        openDoor(atoi(id.c_str()));
    }
}
