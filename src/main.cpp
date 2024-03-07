#define _USE_MATH_DEFINES
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <map>
#include <bitset>
#include <iostream>
#include <set>
#include <algorithm>
using namespace std;

const double EPS = 1e-7;                        // 浮点数精度
const double FRAME_COUNT = 50;                  // 1s帧数
const int TOTAL_FRAME = 15000;                   // 总帧数
//const int LIMIT_BUY_FRAME = TOTAL_FRAME - 150;  // 终止购买物品的帧数
const int MAP_ARRAY_SIZE = 210;                 // 地图数组大小，预留一点空间
const int MAP_REAL_SIZE = 200;                  // 地图的真实大小
//const int MAX_WORK_STATION_SIZE = 101;          // 工作站的最大数量，预留点空间
const int ROBOT_NUM = 10;                       // 机器人的数量
const int BERTH_NUM = 10;                       // 泊位的数量
const double DIS_INTERSECT_EPS = 0.5;           // 相交距离误差
const double AVOID_ANGLE_SPEED_DIFF = M_PI / 8; // 避让时让角速度偏移的差值
const double PREDICT_FRAME = 15;                // 预测的帧数
const double SPEED_DEC_RATE = 0.0000005;        // 速度减小值
const double MIN_ANGLE = 0.08; // 最小角度，用于判断朝向与目标点是否到夹角最小值，以控制不再旋转
const double MAX_DIS = 20000; // 最远距离

//const int n = 200;  // refactor name: MAP_REAL_SIZE
//const int robot_num = 10;  // refactor name: ROBOT_NUM
//const int berth_num = 10;  // refactor name: BERTH_NUM
//const int N = 210;  // refactor name: MAP_ARRAY_SIZE
const vector<int> TYPE_TO_GO[8] = {
        // 物品类型到要前往的工作站
        {7, 6, 5, 4, 3, 2, 1}, // 物品0，表示没有物品，那么去找工作站1-7
        {5, 4, 9},             // 物品1可以卖给5、4、9
        {6, 4, 9},             // 物品2可以卖给6、4、9
        {6, 5, 9},             // 物品3可以卖给6、5、9
        {7, 9},                // 物品4可以卖给7、9
        {7, 9},                // 物品5可以卖给7、9
        {7, 9},                // 物品6可以卖给7、9
        {8, 9}                 // 物品7可以卖给8、9
};

const set<int> TYPE_TO_RECYCLE[10] = {
        // 工作台回收的材料
        {},                   // 工作台0不回收
        {},                   // 工作台1不回收
        {},                   // 工作台2不回收
        {},                   // 工作台3不回收
        {1, 2},               // 工作台4回收1、2
        {1, 3},               // 工作台5回收1、3
        {2, 3},               // 工作台6回收2、3
        {4, 5, 6},            // 工作台7回收4、5、6
        {7},                  // 工作台8回收7
        {1, 2, 3, 4, 5, 6, 7} // 工作台9回收1-7
};

const int STATION_PRODUCE_TIME[10] = {0, 50, 50, 50, 500, 500, 500, 1000, 1, 1}; // 工作台的工作时间

struct Point {
    double x;
    double y;
};

inline double CalcDis(const Point &p1, const Point &p2)
{
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

inline bool IsEq(double x1, double x2, const double eps = EPS)
{
    return abs(x1 - x2) < eps;
}

inline double CalcSpeed(double x, double y)
{
    // 向量(x,y)表示线速度，那么它的速度数值就是向量的模，也就是x和y的平方和开根号。
    // 线速度数值 = √(x² + y1²)
    return sqrt(x * x + y * y);
}

struct Berth
{
    Point p;
    int transport_time;
    int loading_speed;
    Berth(){}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this -> p.x = x;
        this -> p.y = y;
        this -> transport_time = transport_time;
        this -> loading_speed = loading_speed;
    }
};

char g_map[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
char g_ok[5];
int g_frameId;
int g_money;
int g_stationCount;
Berth g_berths[BERTH_NUM + 10];
//vector<Berth *> g_typeToStations[10];
//vector<Berth *> g_stationsToGo[8]; // 物品类型对应去买卖的工作站
int g_recycleTypeCount[8];               // 当前可回收|物品类型的计数
//bitset<8> g_sellLock[MAX_WORK_STATION_SIZE];
//bitset<MAX_WORK_STATION_SIZE> g_buyLock;
int g_buyCount;

struct Robot;

vector<Robot> g_robots(ROBOT_NUM + 10);

struct Robot
{
    int id;
    int nearWorkStation; // -1 表示没有靠近工作站
    int productType;     // 0表示没有携带物品
    double timeRate;     // 时间系数
    double crashRate;    // 碰撞系数
    double v;            // 当前速度
    double vX;           // 线速度x轴分量
    double vY;           // 线速度y轴分量
    double w;            // 当前转动速度
    double r;            // 当前半径
    double mass;         // 质量
    double theta;        // 当前朝向
    Point p;             // 当前坐标

    double eV;                  // 期望速度
    double eW;                  // 期望角速度
    bool buy;                   // 是否买
    bool sell;                  // 是否卖
    bool isNearTargetStation;   // 是否靠近目标工作站
    double scoreToTarget;       // 到目标工作站的价值
    double disToTarget;         // 到目标工作站的距离
    Berth *targetBerth; // 目标泊位

    Point nP; // 预测机器人之后的位置

    static constexpr double RADIUS = 0.45;
    static constexpr double RADIUS_WITH_PRODUCT = 0.53;
    static constexpr double DENSITY = 20.0;
    static constexpr double MASS = M_PI * RADIUS * RADIUS * DENSITY;                                        // 质量
    static constexpr double MASS_WITH_PRODUCT = M_PI * RADIUS_WITH_PRODUCT * RADIUS_WITH_PRODUCT * DENSITY; // 质量
    static constexpr double MAX_FORWARD_SPEED = 6.0;         // 最大前进速度
    static constexpr double MAX_BACKWARD_SPEED = 2.0;        // 最大后退速度
    static constexpr double MAX_ANGLE_SPEED = M_PI + 0.3125; // 最大旋转速度
    static constexpr double MAX_FORCE = 250.0;               // 最大牵引力
    static constexpr double MAX_TORQUE = 50.0;               // 最大力矩

    int goods;
    int status;
    Robot() {}
    Robot(int startX, int startY) {
        p.x = startX;
        p.y = startY;
    }

    void Init()
    {
        v = CalcSpeed(vX, vY);
        r = (productType ? RADIUS_WITH_PRODUCT : RADIUS);
        mass = (productType ? MASS_WITH_PRODUCT : MASS);
        eV = 0.;
        eW = 0.;
        buy = false;
        sell = false;
        isNearTargetStation = false;
        scoreToTarget = MAX_DIS;
        disToTarget = MAX_DIS;
        targetBerth = nullptr;
    }

//    void findSuitableStation()
//    {
//        const auto &stationsToGo = g_stationsToGo[productType];
//        for (auto *station : stationsToGo) {
//            if (productType) {
//                if (station->rawMaterialGrid[productType] || g_sellLock[station->id][productType]) {
//                    continue;
//                }
//            } else {
//                if (!station->hasProduct || g_recycleTypeCount[station->type] <= 0 ||
//                    (g_buyLock[station->id] && g_buyCount > 0)) { //|| CalcDis(p, station->p) > 25))) {
//                    continue;
//                }
//            }
//
//            static int score1[10] = {0, 1, 1, 1, 10, 10, 10, 10, 1, 1};
//
//            // TODO: 还得考虑一下工作台的工作时间 ？
//            auto dis = CalcDis(station->p, p);
//            auto scoreTmp = dis;
//            if (productType) {
//                int score = 0;
//                for (auto e : TYPE_TO_RECYCLE[station->type]) {
//                    score += station->rawMaterialGrid[e] * score1[station->type];
//                }
//                scoreTmp -= score;
//                scoreTmp = max(0.1, scoreTmp);
//                // } else {
//                //     dis -= station->type / 2.;
//                //     dis = max(0.1, dis);
//            }
//
//            if (scoreTmp < scoreToTarget) {
//                scoreToTarget = scoreTmp;
//                disToTarget = dis;
//                targetBerth = station;
//            }
//        }
//        if (targetStation != nullptr) {
//            if (productType) {
//                g_sellLock[targetStation->id][productType] = 1; // 要卖给这个工作站， 占个位置
//                sell = true;
//            } else {
//                if (g_frameId <= LIMIT_BUY_FRAME) {
//                    --g_recycleTypeCount[targetStation->type]; // 要买这个工作站的物品，消费掉一个回收额度
//                    g_buyLock[targetStation->id] = 1;
//                    --g_buyCount;
//                    buy = true;
//                }
//            }
//        }
//    }

//    void CalcForwardSpeedAndRotateSpeed()
//    {
//        isNearTargetStation = (nearWorkStation == targetStation->id);
//        if (isNearTargetStation) { // 如果已经到达目标点，停止运动
//            eV = 0.;
//            eW = 0.;
//            return;
//        }
//        auto angle = CalcNeedRotateAngle(targetStation->p);
//        auto absAngle = abs(angle);
//        const double maxRotateSpeed = (angle > 0 ? MAX_ANGLE_SPEED : -MAX_ANGLE_SPEED);
//        const double maxSpeed = min(scoreToTarget / 0.0525, MAX_FORWARD_SPEED);
//        if (absAngle < MIN_ANGLE) { // 如果朝向和目标点的夹角很小，直接全速前进
//            eV = MAX_FORWARD_SPEED;
//            eW = 0.;
//        } else {
//            if (absAngle > M_PI / 2) {
//                // 角度太大，全速扭转
//                // 速度控制小一点，避免靠近不了工作台
//                eV = MAX_FORWARD_SPEED * 0.2;
//                eW = maxRotateSpeed;
//            } else {
//                eV = MAX_FORWARD_SPEED * cos(absAngle); // 前进速度随角度变小而变大
//                eW = maxRotateSpeed * sin(absAngle);    // 旋转速度随角度变小而变小
//            }
//        }
//    }

    void CalcNp()
    {
        nP.x = p.x + v * cos(theta) * PREDICT_FRAME / FRAME_COUNT;
        nP.y = p.y + v * sin(theta) * PREDICT_FRAME / FRAME_COUNT;
    }

private:
    double CalcNeedRotateAngle(const Point &point) const
    {
        if (IsEq(p.x, point.x) && IsEq(p.y, point.y)) {
            return 0.;
        }
        const double angle = atan2(point.y - p.y, point.x - p.x);
        const double rotation = angle - theta;
        // 如果要旋转的弧度值大于π或小于-π，取补角
        if (rotation > M_PI) {
            return rotation - 2 * M_PI;
        }
        if (rotation < -M_PI) {
            return rotation + 2 * M_PI;
        }
        return rotation;
    }
};


struct Boat
{
    int num, pos, status;
}boat[10];

int money, boat_capacity, id;
//char g_map[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
int gds[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];

inline Point GetPoint(double i, double j)
{
    return {0.5 * j + 0.25, 49.75 - 0.5 * i};
}

void Init()
{
    for(int i = 1; i <= MAP_REAL_SIZE; i ++)
        scanf("%s", g_map[i] + 1);
//    for(int i = 1; i <= MAP_REAL_SIZE; i ++)
//        printf("%s", g_map[i]);

    for(int i = 0; i < BERTH_NUM; i ++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &g_berths[id].p.x, &g_berths[id].p.y, &g_berths[id].transport_time, &g_berths[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

int Input()
{
    scanf("%d%d", &id, &money);
    int num;
    scanf("%d", &num);
    for(int i = 1; i <= num; i ++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
    }
    for(int i = 0; i < ROBOT_NUM; i ++)
    {
        int sts;
        scanf("%d%d%d%d", &g_robots[i].goods, &g_robots[i].p.x, &g_robots[i].p.y, &sts);
    }
    for(int i = 0; i < 5; i ++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
    return id;
}

int main()
{
    ios::sync_with_stdio(false);
    cout.tie(nullptr);
    setbuf(stdout, nullptr);
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        for(int i = 0; i < ROBOT_NUM; i ++)
            printf("move %d %d\n", i, rand() % 4);
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
