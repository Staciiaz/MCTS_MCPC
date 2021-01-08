#include <iostream>
#include <iomanip>
#include <fstream>
#include <set>
#include <map>
#include <vector>
#include <cmath>
#include <climits>

using namespace std;

class Track {
public:
    pair<double, double> position;
    double speed;
    double bearing;
    Track(pair<double, double> position, double speed, double bearing) {
        this->position = position;
        this->speed = speed;
        this->bearing = bearing;
    }
};

class CollisionResult {
public:
    bool intersection;
    pair<double, double> point;
    double bearing;
    double speed;
    double time;
    CollisionResult(bool intersection) {
        this->intersection = intersection;
    }
};

double find_bearing(pair<double, double> point1, pair<double, double> point2)
{
    const double degToRad = M_PI / 180;
    double lat1 = point1.first * degToRad;
    double lat2 = point2.first * degToRad;
    double lng1 = point1.second * degToRad;
    double lng2 = point2.second * degToRad;
    double dLng = lng2 - lng1;
    if (abs(dLng) > M_PI) {
        dLng = dLng > 0 ? -(2 * M_PI - dLng) : (2 * M_PI + dLng);
    }
    double dA = log(tan(lat2 / 2 + M_PI / 4) / tan(lat1 / 2 + M_PI / 4));
    double theta = atan2(dLng, dA);
    double bearing = theta / degToRad;
    return ((int) bearing % 360) + (bearing < 0 ? 360 : 0) + (bearing - (int) bearing);
}

double find_distance(pair<double, double> point1, pair<double, double> point2)
{
    const double R = 6371.0;
    const double degToRad = M_PI / 180;
    double lat1 = point1.first * degToRad;
    double lat2 = point2.first * degToRad;
    double lng1 = point1.second * degToRad;
    double lng2 = point2.second * degToRad;
    double dLat = lat2 - lat1;
    double dLng = lng2 - lng1;
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLng / 2.0) * sin(dLng / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double d = R * c;
    return d;
}

pair<double, double> find_distance_point(pair<double, double> point, double bearing, double distance) {
    const double R = 6371.0;
    const double degToRad = M_PI / 180;
    const double radiusEarthKilometres = 6371.01;
    double distRatio = distance / radiusEarthKilometres;
    double distRatioSine = sin(distRatio);
    double distRatioCosine = cos(distRatio);
    double startLatRad = point.first * degToRad;
    double startLonRad = point.second * degToRad;
    double bearingRad = bearing * degToRad;
    double startLatCos = cos(startLatRad);
    double startLatSin = sin(startLatRad);
    double endLatRads = asin((startLatSin * distRatioCosine) + (startLatCos * distRatioSine * cos(bearingRad)));
    double endLonRads = startLonRad + atan2(sin(bearingRad) * distRatioSine * startLatCos, distRatioCosine - startLatSin * sin(endLatRads));
    return make_pair(endLatRads / degToRad, endLonRads / degToRad);
}

CollisionResult collision_calculate(Track track1, Track track2) {
    set<double> bearings;
    map<double, CollisionResult> results;
    const double degToRad = M_PI / 180;
    const int bearingRotation = 360;
    const int speedAdjust = 500;
    int startBearing = ((int) track1.bearing - bearingRotation + 360) % 360;
    int endBearing = ((int) track1.bearing + bearingRotation + 360) % 360;
    if (startBearing < endBearing) {
        for (int i = startBearing; i <= endBearing; i++) {
            int bearing = i % 360;
            bearings.insert(bearing);
        }
    } else {
        for (int i = startBearing; i < 360; i++) {
            bearings.insert(i);
        }
        for (int i = 0; i < endBearing; i++) {
            bearings.insert(i);
        }
    }
    double bearing = 0.0;
    while (bearing < 360.0) {
        double lat1 = track1.position.first * degToRad;
        double lat2 = track2.position.first * degToRad;
        double lng1 = track1.position.second * degToRad;
        double lng2 = track2.position.second * degToRad;
        double dLat = lat2 - lat1;
        double dLng = lng2 - lng1;
        double theta13 = bearing * degToRad;
        double theta23 = track2.bearing * degToRad;
        double delta12 = 2 * asin(sqrt(pow(sin(dLat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dLng / 2), 2)));
        double cosThetaA = (sin(lat2) - sin(lat1) * cos(delta12)) / (sin(delta12) * cos(lat1));
        double cosThetaB = (sin(lat1) - sin(lat2) * cos(delta12)) / (sin(delta12) * cos(lat2));
        double thetaA = acos(min(max(cosThetaA, -1.0), 1.0));
        double thetaB = acos(min(max(cosThetaB, -1.0), 1.0));
        double theta12 = sin(dLng) > 0 ? thetaA : 2 * M_PI - thetaA;
        double theta21 = sin(dLng) > 0 ? 2 * M_PI - thetaB : thetaB;
        double alpha1 = theta13 - theta12;
        double alpha2 = theta21 - theta23;
        if ((sin(alpha1) == 0 && sin(alpha2) == 0) || sin(alpha1) * sin(alpha2) < 0) {
            //cout << "Bearing " << bearing << " unable to collision" << endl;
        } else {
            double cosAlpha3 = -cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2) * cos(delta12);
            double delta13 = atan2(sin(delta12) * sin(alpha1) * sin(alpha2), cos(alpha2) + cos(alpha1) * cosAlpha3);
            double lat3 = asin(sin(lat1) * cos(delta13) + cos(lat1) * sin(delta13) * cos(theta13));
            double dLat13 = atan2(sin(theta13) * sin(delta13) * cos(lat1), cos(delta13) - sin(lat1) * sin(lat3));
            double lng3 = lng1 + dLat13;
            pair<double, double> intersectPoint = make_pair(lat3 / degToRad, lng3 / degToRad);
            double distance1 = find_distance(track1.position, intersectPoint);
            double distance2 = find_distance(track2.position, intersectPoint);
            double time = distance2 / track2.speed;
            double speed1 = distance1 / time;
            auto result = CollisionResult(true);
            result.point = intersectPoint;
            result.bearing = bearing;
            result.speed = speed1;
            result.time = time;
            if (abs(find_bearing(track2.position, result.point) - track2.bearing) < 0.01) {
                cout << "Test Bearing 2 : " << fixed << setprecision(6) << find_bearing(track2.position, result.point) << ", " << track2.bearing << endl;
                cout << fixed << setprecision(6) << "Bearing " << bearing << " intersect at " << result.point.first << " " << result.point.second << endl;
                cout << "Bearing of fighter and collision point : " << find_bearing(track1.position, result.point) << endl;
                cout << "Fighter speed : " << fixed << setprecision(6) << result.speed << endl;
                results.insert(make_pair(bearing, result));
            } else {
                //cout << "Bearing " << bearing << " unable to collision" << endl;
            }
        }
        bearing += 0.001;
    }
    if (!results.empty()) {
        double sBearing = -1;
        double sSpeed = INT_MAX;
        for (auto it : results) {
            double bearing = it.first;
            CollisionResult result = it.second;
            double speedDifference = abs(result.speed - track1.speed);
            if (speedDifference < speedAdjust && speedDifference < sSpeed) {
                sBearing = bearing;
                sSpeed = speedDifference;
            }
        }
        if (sBearing != -1) {
            return results.at(sBearing);
        }
    }
    return CollisionResult(false);
}

int main() {
    double lat, lng, spd, bearing;
    vector<Track> targets;
    ifstream fin("../test_track.txt");
    fin >> lat >> lng >> spd >> bearing;
    Track fighter = Track(make_pair(lat, lng), spd, bearing);
    while (!fin.eof()) {
        fin >> lat >> lng >> spd >> bearing;
        Track target = Track(make_pair(lat, lng), spd, bearing);
        targets.push_back(target);
    }
    CollisionResult result = collision_calculate(fighter, targets[0]);
    if (result.intersection) {
        cout << "Point : " << result.point.first << " " << result.point.second << endl;
        cout << "Bearing : " << result.bearing << endl;
        cout << "Speed : " << result.speed << endl;
        cout << "Time : " << result.time << endl;
    } else {
        cout << "Impossible" << endl;
    }
    return 0;
}