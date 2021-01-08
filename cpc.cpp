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
    Track() {

    }
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

class Node {
public:
    int id;
    Track track = Track();
    CollisionResult result = CollisionResult(false);
    int win;
    int sim;
    Node* parent;
    map<int, Node*> children;
    Node(int id, Node* parent, Track track) {
        this->id = id;
        this->track = track;
        this->result = CollisionResult(false);
        this->win = 0;
        this->sim = 0;
        this->parent = parent;
    }
};

enum FocusResult {
    TIME, DISTANCE
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
    return ((int) bearing % 360) + (bearing < 0 ? 360 : 0);
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
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLng/2) * sin(dLng/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
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
    const int speedAdjust = 2000;
    const double bearingPrecision = 1.0;
    int startBearing = ((int) track1.bearing - bearingRotation + 360) % 360;
    int endBearing = ((int) track1.bearing + bearingRotation + 360) % 360;
    if (startBearing < endBearing) {
        double bearing = startBearing;
        while (bearing <= endBearing) {
            bearings.insert(bearing);
            bearing += bearingPrecision;
        }
    } else {
        double bearing = startBearing;
        while (bearing < 360) {
            bearings.insert(bearing);
            bearing += bearingPrecision;
        }
        bearing = 0;
        while (bearing <= endBearing) {
            bearings.insert(bearing);
            bearing += bearingPrecision;
        }
    }
    for (auto bearing : bearings) {
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
        if (!((sin(alpha1) == 0 && sin(alpha2) == 0) || sin(alpha1) * sin(alpha2) < 0)) {
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
                results.insert(make_pair(bearing, result));
            }
        }
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

void mult_collision_calculate(Track fighter, vector<Track> targets, const vector<int> &path) {
    double time = 0, distance = 0;
    set<int> visited;
    ofstream fout("../result_cpc.txt");
    fout.precision(7);
    fout << fixed;
    fout << 1 << endl;
    fout << fighter.position.first << " " << fighter.position.second << " " << fighter.speed << " " << fighter.bearing << endl;
    fout << targets.size() << endl;
    for (int i = 0; i < targets.size(); i++) {
        fout << targets[i].position.first << " " << targets[i].position.second << " " << targets[i].speed << " " << targets[i].bearing << endl;
    }
    fout << path.size() << endl;
    for (int trackIndex : path) {
        Track track = targets[trackIndex];
        CollisionResult result = collision_calculate(fighter, track);
        Track oldFighter = fighter;
        if (result.intersection) {
            cout << "Track #" << trackIndex << endl;
            fighter.position = result.point;
            fighter.bearing = result.bearing;
            fighter.speed = result.speed;
            time += result.time;
            distance += result.speed * result.time;
            for (int j = 0; j < targets.size(); j++) {
                if (visited.find(j) == visited.end()) {
                    Track& jTrack = targets[j];
                    jTrack.position = find_distance_point(jTrack.position, jTrack.bearing, jTrack.speed * result.time);
                }
            }
            fout << "1" << endl;
            fout << oldFighter.position.first << " " << oldFighter.position.second << " " << oldFighter.speed << " " << oldFighter.bearing << endl;
            fout << track.position.first << " " << track.position.second << " " << track.speed << " " << track.bearing << endl;
            fout << result.point.first << " " << result.point.second << endl;
            cout << "Fighter : " << oldFighter.position.first << " " << oldFighter.position.second << endl;
            cout << "Target : " << track.position.first << " " << track.position.second << endl;
            cout << "Point : " << result.point.first << " " << result.point.second << endl;
            cout << "Bearing : " << result.bearing << endl;
            cout << "Speed : " << result.speed << endl;
            cout << "Time : " << result.time << endl;
        } else {
            fout << "0" << endl;
            fout << oldFighter.position.first << " " << oldFighter.position.second << " " << oldFighter.speed << " " << oldFighter.bearing << endl;
            fout << track.position.first << " " << track.position.second << " " << track.speed << " " << track.bearing << endl;
            cout << "Impossible" << endl;
        }
        visited.insert(trackIndex);
    }
    cout << "Estimated Time : " << time << endl;
    cout << "Total Distance: " << distance << endl;
}

vector<int> mcts(Track fighter, const vector<Track>& targets, FocusResult focusResult) {
    const double c = 1;
    double minimumTime = INT_MAX;
    double minimumDistance = INT_MAX;
    unsigned int maximumVertex = 0;
    vector<Node*> path;
    vector<int> minimumPath;
    Node* minimumNode = nullptr;
    int totalSimulation = 0;
    Node root = Node(-1, nullptr, fighter);
    for (int itr = 0; itr < 100; itr++) {
        set<int> visited;
        double time = 0, distance = 0;
        Node *curr = &root;
        Track currentFighter = fighter;
        vector<Track> currentTargets;
        for (auto target : targets) {
            currentTargets.push_back(target);
        }
        path.push_back(curr);
        do {
            int selectedTrack = -1;
            Node *selectionNode = nullptr;
            double best_ucb = INT_MIN;
            for (int i = 0; i < targets.size(); i++) {
                if (visited.find(i) == visited.end()) {
                    Node* expansionNode;
                    if (curr->children.find(i) != curr->children.end()) {
                        expansionNode = curr->children.at(i);
                    } else {
                        Track target = currentTargets[i];
                        expansionNode = new Node(i, curr, target);
                        expansionNode->result = collision_calculate(currentFighter, expansionNode->track);
                        curr->children.insert(make_pair(i, expansionNode));
                    }
                    if (expansionNode->result.intersection) {
                        double nodeMean = (double) expansionNode->win / expansionNode->sim;
                        double mctsFactor = c * sqrt(log(totalSimulation) / expansionNode->sim);
                        double ucb = nodeMean + mctsFactor;
                        if (isnan(ucb)) {
                            ucb = INFINITY;
                        }
                        if (ucb > best_ucb) {
                            best_ucb = ucb;
                            selectionNode = expansionNode;
                            selectedTrack = i;
                        }
                    }
                }
            }
            if (selectedTrack != -1) {
                visited.insert(selectedTrack);
                path.push_back(selectionNode);
                time += selectionNode->result.time;
                distance += selectionNode->result.speed * selectionNode->result.time;
                currentFighter.position = selectionNode->result.point;
                currentFighter.speed = selectionNode->result.speed;
                currentFighter.bearing = selectionNode->result.bearing;
                for (auto &track : currentTargets) {
                    track.position = find_distance_point(track.position, track.bearing, track.speed * selectionNode->result.time);
                }
            }
            curr = selectionNode;
        } while (curr != nullptr);
        for (auto backpropagationNode : path) {
            if (path.size() - 1 == targets.size()) {
                backpropagationNode->win++;
            }
            backpropagationNode->sim++;
            totalSimulation++;
        }
        if (focusResult == TIME) {
            if (path.size() - 1 >= maximumVertex && time < minimumTime) {
                minimumTime = time;
                maximumVertex = path.size() - 1;
                minimumNode = path[maximumVertex];
            }
        } else if (focusResult == DISTANCE) {
            if (path.size() - 1 >= maximumVertex && distance < minimumDistance) {
                minimumDistance = distance;
                maximumVertex = path.size() - 1;
                minimumNode = path[maximumVertex];
            }
        }
        path.clear();
    }
    Node *traverseNode = minimumNode;
    while (traverseNode != &root) {
        minimumPath.insert(minimumPath.begin(), traverseNode->id);
        traverseNode = traverseNode->parent;
    }
    return minimumPath;
}

int main() {
    double lat, lng, spd, bearing;
    vector<Track> targets;
    ifstream fin("../track.txt");
    fin >> lat >> lng >> spd >> bearing;
    Track fighter = Track(make_pair(lat, lng), spd, bearing);
    while (!fin.eof()) {
        fin >> lat >> lng >> spd >> bearing;
        Track target = Track(make_pair(lat, lng), spd, bearing);
        targets.push_back(target);
    }
    vector<int> path = mcts(fighter, targets, TIME);
    mult_collision_calculate(fighter, targets, path);
    cout << "Order of Collision : ";
    for (auto it : path) {
        cout << (it+1) << " ";
    }
    cout << endl;
    return 0;
}