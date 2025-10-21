//
//
//

#ifndef VEHICLE_H
#define VEHICLE_H
#include<string>



class Vehicle {
private:
    // private attributes
    double fuelUsed;
    double DistanceTravelled;
    double fuelPerMile;
    std::string vehicleType;
    double fuelCapacity;
    int speed;
    int direction;
public:
    //constructor
    Vehicle(std::string type);
    //getters
    std::string getCarType();
    double getFuelUsed();
    double getDistanceTravelled();
    double  getFuelPerMile() const;
    double getFuelCapacity();
    int getSpeed();
    int getDirection();


    //setters
    void setFuelUsed(double fuelUsed);
    void setDistanceTravelled(double distanceTravelled);
    void setFuelPerMile(double fuelPerMile);
    void setCarType(std::string type);
    void setFuelCapacity(double fuelCapacity);
    void setdirection(int direction);

    void breaking(int amount);
    void accelerating(int amount);




};



#endif //VEHICLE_H
