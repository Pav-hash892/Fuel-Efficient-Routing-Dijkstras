//
// Created by Sai Pavan Kukkadapu on 6/2/25.
//

#include "Vehicle.h"

#include <algorithm>

// Constructor with type-based defaults
Vehicle::Vehicle(std::string type)
    : vehicleType(type),
      fuelUsed(0.0),
      DistanceTravelled(0.0),
      direction(0),
      speed(0) {

    if (type == "sedan") {
        fuelCapacity = 50.0;
        fuelPerMile = 0.066; // ~15 km/L
        speed = 120;
    } else if (type == "truck") {
        fuelCapacity = 120.0;
        fuelPerMile = 0.166; // ~6 km/L
        speed = 90;
    } else if (type == "suv") {
        fuelCapacity = 65.0;
        fuelPerMile = 0.083; // ~12 km/L
        speed = 110;
    } else if (type == "bus") {
        fuelCapacity = 200.0;
        fuelPerMile = 0.25; // ~4 km/L
        speed = 80;
    } else {
        fuelCapacity = 50.0;
        fuelPerMile = 0.066;
        speed = 100;
    }
}

// Getters
std::string Vehicle::getCarType() { return vehicleType; }
double Vehicle::getFuelUsed() { return fuelUsed; }
double Vehicle::getDistanceTravelled() { return DistanceTravelled; }
double  Vehicle::getFuelPerMile() const { return fuelPerMile; }
double Vehicle::getFuelCapacity() { return fuelCapacity; }
int Vehicle::getSpeed() { return speed; }
int Vehicle::getDirection() { return direction; }

// Setters
void Vehicle::setFuelUsed(double fuel) {
    fuelUsed = std::clamp(fuel, 0.0, fuelCapacity);
}

void Vehicle::setDistanceTravelled(double distance) {
    DistanceTravelled = std::max(0.0, distance);
}

void Vehicle::setFuelPerMile(double fpm) {
    fuelPerMile = std::max(0.0, fpm);
}

void Vehicle::setCarType(std::string type) {
    vehicleType = type;
}

void Vehicle::setFuelCapacity(double capacity) {
    fuelCapacity = std::max(0.0, capacity);
}

void Vehicle::setdirection(int dir) {
    direction = dir % 360;
}

// Behavior
void Vehicle::breaking(int amount) {
    speed = std::max(0, speed - amount);
}

void Vehicle::accelerating(int amount) {
    speed += amount;
}
