#include "../include/waypoint_maker.h"

#define SWAP(x, y, t) ((t) = (x), (x) = (y), (y) = (t))

using namespace std;

#define X_CMP 1
#define Y_CMP 2
#define D_CMP 3

typedef struct Object {
    double centerX;
    double centerY;
    double centerZ;
    double lengthX;
    double lengthY;
    double lengthZ;
    double distance;
    double volume;
}Object;

typedef struct ObjectArray
{
    int size = 0;
    Object objectArray[100];
}ObjectArray;

typedef struct
{
    int size = 0;
    Object* cone[30];
}ConeArray;

int partition(Object list[], int left, int right, int cmp) {
    switch (cmp) {
        case X_CMP: {
            double pivot = list[left].centerX;
            int low = left;
            int high = right + 1;
            Object temp;

            do {
                do {
                    low++;
                } while (list[low].centerX < pivot);
                do {
                    high--;
                } while (list[high].centerX > pivot);

                if (low < high) {
                    SWAP(list[low], list[high], temp);
                }

            } while (low < high);

            SWAP(list[left], list[high], temp);
            
            return high;
            }
        case Y_CMP: {
            double pivot = list[left].centerY;
            int low = left;
            int high = right + 1;
            Object temp;

            do {
                do {
                    low++;
                } while (list[low].centerY < pivot);
                do {
                    high--;
                } while (list[high].centerY > pivot);

                if (low < high) {
                    SWAP(list[low], list[high], temp);
                }

            } while (low < high);

            SWAP(list[left], list[high], temp);

            return high;
            }
        case D_CMP: {
            double pivot = list[left].distance;
            int low = left;
            int high = right + 1;
            Object temp;

            do {
                do {
                    low++;
                } while (list[low].distance < pivot);
                do {
                    high--;
                } while (list[high].distance > pivot);

                if (low < high) {
                    SWAP(list[low], list[high], temp);
                }

            } while (low < high);

            SWAP(list[left], list[high], temp);

            return high;
            }
        default: {
            cout << "SORT PARAMETER ERROR \n";
            return 0;
            }
    }
}

void quickSort(Object array[], int left, int right, int cmp) {
    if (left < right) {
        int p = partition(array, left, right, cmp);
        quickSort(array, left, p - 1, cmp);
        quickSort(array, p + 1, right, cmp);
    }
}

double getDistanceObjectToObject(const Object &obj1, const Object &obj2) {
    return sqrt( pow(obj1.centerX - obj2.centerX, 2) + pow(obj1.centerY - obj2.centerY, 2) );
}

class WaypointMaker{
    // attribute
    ObjectArray objects; // subscribe msg data ( lidar_waypoint_maker 토픽 구독해서 받는 정보를 저장하는 객체 )

    Object leftPivot; // pivot ( 좌측 cone을 분류 할 때 기준이 되는 pivot 객체 )
    Object rightPivot; // pivot ( 우측 cone을 분류 할 때 기준이 되는 pivot 객체 )
    
    ConeArray leftCones; // cone segmentation ( 좌측 cone의 정보를 담는 객체 )
    ConeArray rightCones; // cone segmentation ( 좌측 cone의 정보를 담는 객체 )
    /* 알아야 하는 중요한 정보 : ConeArray 타입은 위에 구조체로 선언되어 있는데 다음과 같습니다.
    // int size         원소의 수가 총 몇개인지 저장되는 변수
    // Object* cone[30] Object를 가리키는 포인터를 담는 배열 
    // 포인터 배열을 이용하여 연산속도를 효과적으로 줄이게 됩니다.
    */

    waypoint_maker::Waypoint waypointInfoMsg; // waypoint msg ( Waypoint를 발행하기 위한 메시지타입 객체 )

    // ros attribute
    ros::NodeHandle nh; // 객체를 생성만 해서 노드를 실행시키기 위해서 가지는 멤버변수
    ros::Subscriber mainSub; // 이 객체가 중심적으로 Subscribe하고 객체의 제일 중요한 함수를 Callback 하는 기능을 수행해서 이름을 mainSub라고 했습니다.
    ros::Subscriber sensorFusionResultSub; 

    ros::Publisher waypointInfoPub; // 이름 그대로를 수행하는 Publisher
    ros::Publisher visualizeConePub; // 이름 그대로를 수행하는 Publisher
    ros::Publisher visualizeWaypointInfoMsgPub; // 이름 그대로를 수행하는 Publisher
    ros::Publisher visualizePivotPub; // 이름 그대로를 수행하는 Publisher
    
    public:
    
    double xMinRubberCone;
    double xMaxRubberCone;
    double yMinRubberCone;
    double yMaxRubberCone;
    double zMinRubberCone;
    double zMaxRubberCone;

    // constructor
    WaypointMaker() {
        /**
         * @brief 
         * 객체가 생성될 때 임의의 피봇 값을 설정하지 않으면 쓰레기 값이 들어가기 때문에 Critical Error가 발생합니다. 생성자로 값을 초기화 시킵니다.
         * ROS Subscriber 설정과 Publisher를 설정합니다
         */
        
        leftPivot.centerX = 0.0;
        leftPivot.centerY = 1.0;
        leftPivot.centerZ = 0.0;
        rightPivot.centerX = 0.0;
        rightPivot.centerY = -1.0;
        rightPivot.centerZ = 0.0;

        // subscribe
        mainSub = nh.subscribe("/traffic_cone", 1, &WaypointMaker::mainCallback, this);
        sensorFusionResultSub = nh.subscribe("/perfect_cone", 1, &WaypointMaker::sensorFusionCallback, this);

        visualizeConePub = nh.advertise<visualization_msgs::MarkerArray>("/cone_marker", 0.001);
        waypointInfoPub = nh.advertise<waypoint_maker::Waypoint>("/waypoint_info", 0.001);
        visualizeWaypointInfoMsgPub = nh.advertise<visualization_msgs::MarkerArray>("/waypoint_marker_fake", 0.001);
        visualizePivotPub = nh.advertise<visualization_msgs::MarkerArray>("/pivot_marker", 0.001);
    }

    // method
    void mainCallback(const lidar_trafficcone_detection::Trafficcone& msg); // 이 객체의 전체 로직을 수행하는 함수입니다.
    void setObjectInfo(const lidar_trafficcone_detection::Trafficcone& msg); // object_info 정보를 받아 값을 objects에 저장하고 X값 기준 오름차순으로 정렬합니다.
    void sensorFusionCallback(const waypoint_maker::clustering_points& msg);
    // visualize method
    void visualizeLeftRightCone(); // 이름 그대로를 수행하는 Rviz 시각화 함수
    void visualizeWaypointInfoMsg(); // 이름 그대로를 수행하는 Rviz 시각화 함수

    // dynamic reconfigure
    // void cfgCallback(waypoint_maker::waypointMakerConfig &config, int32_t level);
};

void WaypointMaker::mainCallback(const lidar_trafficcone_detection::Trafficcone& msg) {
    setObjectInfo(msg);
    // setLeftRightConeInfo();
    // setWaypointInfo();

    //1006
    // visualizeLeftRightCone();
    // visualizeWaypointInfoMsg();
    // visualizePivot();
}

void WaypointMaker::sensorFusionCallback(const waypoint_maker::clustering_points& msg) {
        std::vector<std::pair<double, double>> leftCones;
        std::vector<std::pair<double, double>> rightCones;

        for (size_t i = 0; i < msg.x.size(); ++i) {
            if (i < msg.label.size()) { 
                if (msg.label[i] == 0) { // yellow
                    leftCones.push_back(std::make_pair(msg.x[i], msg.y[i]));
                } else if (msg.label[i] == 1) { // blue
                    rightCones.push_back(std::make_pair(msg.x[i], msg.y[i]));
                }
            }
        }

        // x 좌표를 기준으로 오름차순 정렬
        std::sort(leftCones.begin(), leftCones.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.first < b.first;
            });

        std::sort(rightCones.begin(), rightCones.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.first < b.first;
            });
        
        waypointInfoMsg.cnt = 0;

        int leftSize = leftCones.size();
        int rightSize = rightCones.size();
        int minSize = std::min(leftSize, rightSize);
        minSize = std::min(minSize, 100);  // boost::array의 크기가 100이라고 가정

        for (int i = 0; i < minSize; ++i) {
            double waypointX = (leftCones[i].first + rightCones[i].first) / 2.0;
            double waypointY = (leftCones[i].second + rightCones[i].second) / 2.0;
            cout << "waypoint X : " << waypointX << "waypoint Y : " << waypointY << endl;
            waypointInfoMsg.x_arr[i] = waypointX;
            waypointInfoMsg.y_arr[i] = waypointY;
            waypointInfoMsg.cnt++;
        }

        // Waypoint 정보 발행
        waypointInfoPub.publish(waypointInfoMsg);
        visualizeWaypointInfoMsg();

    }

void WaypointMaker::setObjectInfo(const lidar_trafficcone_detection::Trafficcone& msg) {
    /**
     * @brief 
     * object_detector가 발행하는 /object_info 토픽 메시지를 받아
     * 값을 objects에 저장하고 X값 기준 오름차순으로 정렬합니다.
     */

    int count = 0;
    for (int i = 0; i < msg.objectCounts; i++) {
        if ( (this->xMinRubberCone < msg.lengthX[i] && msg.lengthX[i] < this->xMaxRubberCone) &&
             (this->yMinRubberCone < msg.lengthY[i] && msg.lengthY[i] < this->yMaxRubberCone) &&
             (this->zMinRubberCone < msg.lengthZ[i] && msg.lengthZ[i] < this->zMaxRubberCone) ) {
            this->objects.objectArray[count].centerX = msg.centerX[i];
            this->objects.objectArray[count].centerY = msg.centerY[i];
            this->objects.objectArray[count].centerZ = msg.centerZ[i];

            this->objects.objectArray[count].lengthX = msg.lengthX[i];
            this->objects.objectArray[count].lengthY = msg.lengthY[i];
            this->objects.objectArray[count].lengthZ = msg.lengthZ[i];
            count++;
        }
    }

    objects.size = count;
    quickSort(this->objects.objectArray, 0, this->objects.size-1, X_CMP);
}

void WaypointMaker::visualizeLeftRightCone() {
    // ROS_INFO("hihihi");

    visualization_msgs::Marker coneObject;
    visualization_msgs::MarkerArray coneObjectArray;

    coneObject.header.frame_id = "velodyne";
    coneObject.header.stamp = ros::Time();
    coneObject.type = visualization_msgs::Marker::CYLINDER; 
    coneObject.action = visualization_msgs::Marker::ADD;

    coneObject.scale.x = 0.5;
    coneObject.scale.y = 0.5;
    coneObject.scale.z = 0.8;

    coneObject.color.a = 0.5;
    coneObject.color.r = 1.0;
    coneObject.color.g = 1.0;
    coneObject.color.b = 0.0;

    coneObject.lifetime = ros::Duration(0.1);

    // cout <<leftCones.size << endl;
    for (int i = 0; i < leftCones.size; i++) {
        coneObject.id = 100 + i;
        coneObject.pose.position.x = leftCones.cone[i]->centerX;
        coneObject.pose.position.y = leftCones.cone[i]->centerY;
        coneObject.pose.position.z = leftCones.cone[i]->centerZ;

        coneObjectArray.markers.emplace_back(coneObject);
    }

    coneObject.color.a = 0.5; 
    coneObject.color.r = 0.0; 
    coneObject.color.g = 0.0;
    coneObject.color.b = 1.0;

    for (int i = 0; i < rightCones.size; i++) {
        coneObject.id = 200 + i; 
        // cout << "right" << endl;

        coneObject.pose.position.x = rightCones.cone[i]->centerX;
        coneObject.pose.position.y = rightCones.cone[i]->centerY;
        coneObject.pose.position.z = rightCones.cone[i]->centerZ;

        coneObjectArray.markers.emplace_back(coneObject);
    }
    visualizeConePub.publish(coneObjectArray);
}

void WaypointMaker::visualizeWaypointInfoMsg() {
    visualization_msgs::Marker waypoint;
    visualization_msgs::MarkerArray waypointArray;
    cout << "waypoint info " << waypointInfoMsg.cnt << endl;
    for (int i = 0; i < waypointInfoMsg.cnt; i++) {
        waypoint.header.frame_id = "velodyne";
        waypoint.header.stamp = ros::Time();
    
        waypoint.id = 200 + i;
        waypoint.type = visualization_msgs::Marker::SPHERE; 
        waypoint.action = visualization_msgs::Marker::ADD;

        waypoint.pose.position.x = waypointInfoMsg.x_arr[i];
        waypoint.pose.position.y = waypointInfoMsg.y_arr[i];
        waypoint.pose.position.z = 0.2;

        waypoint.scale.x = 0.1;
        waypoint.scale.y = 0.1;
        waypoint.scale.z = 0.1;

        waypoint.color.a = 1.0; 
        waypoint.color.r = 1.0; 
        waypoint.color.g = 0.0;
        waypoint.color.b = 0.0;

        waypoint.lifetime = ros::Duration(0.1);
        waypointArray.markers.emplace_back(waypoint);
    }
    visualizeWaypointInfoMsgPub.publish(waypointArray);
}


void cfgCallback(waypoint_maker::waypointMakerConfig &config, WaypointMaker* wm) {
    wm->xMinRubberCone = config.xMinRubberCone;
    wm->xMaxRubberCone = config.xMaxRubberCone;
    wm->yMinRubberCone = config.yMinRubberCone;
    wm->yMaxRubberCone = config.yMaxRubberCone;
    wm->zMinRubberCone = config.zMinRubberCone;
    wm->zMaxRubberCone = config.zMaxRubberCone;
}

int main(int argc, char **argv) {
    // cout << "!!!!!!!!!!!!!!!!1" << endl;
    ros::init(argc, argv, "waypoint_maker");

    WaypointMaker waypointMaker;
    
    dynamic_reconfigure::Server<waypoint_maker::waypointMakerConfig> server;
    dynamic_reconfigure::Server<waypoint_maker::waypointMakerConfig>::CallbackType f;
    f = boost::bind(&cfgCallback, _1, &waypointMaker);
    server.setCallback(f);
    
    ros::spin();

    // while(ros::ok()){
    //     waypointMaker.visualizeLeftRightCone();
    // }
    return 0;
}