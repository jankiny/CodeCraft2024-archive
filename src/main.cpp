#define _USE_MATH_DEFINES
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
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
const int BOAT_NUM = 10;                        // 轮船的数量
const int BERTH_NUM = 10;                       // 泊位的数量
const int MAX_GOOD_NUM = MAP_REAL_SIZE * MAP_REAL_SIZE + 10;    // 货物的最大数量，预留点空间
//const double DIS_INTERSECT_EPS = 0.5;           // 相交距离误差
//const double AVOID_ANGLE_SPEED_DIFF = M_PI / 8; // 避让时让角速度偏移的差值
const double PREDICT_FRAME = 15;                // 预测的帧数
//const double SPEED_DEC_RATE = 0.0000005;        // 速度减小值
//const double MIN_ANGLE = 0.08; // 最小角度，用于判断朝向与目标点是否到夹角最小值，以控制不再旋转
//const double MAX_DIS = 20000; // 最远距离
const int DIRECTION[4] = {
        0,  // 右移一格
        1,  // 左移一格
        2,  // 上移一格
        3   // 下移一格
};

// 方向数组，用于表示上下左右四个方向
const int DIRECTION_TO_GO[4][2] = {
        {0, 1},     // 右移一格
        {0, -1},    // 左移一格
        {-1, 0},    // 上移一格
        {1, 0}      // 下移一格
};

struct Point {
    int x;
    int y;

    Point(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator==(const Point& other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point& other) const
    {
        return !(*this == other);
    }
};

struct Berth
{
    int id;
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

struct Boat
{
    int capacity;
    int pos;        // 表示目标泊位
    int status;     // 状态(0表示运输中；1表示正常运行状态，即装货中或运输完成；2表示泊位外等待)
};

char g_map[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
char g_ok[100];
int g_frameId;
int g_money;
int g_boatCapacity;
Berth g_berths[BERTH_NUM];
vector<Point> berths_point; // Init时将Berth的位置进行打包，便于在创建Good时计算最近的Berth
Boat g_boats[BOAT_NUM];
//vector<Berth *> g_typeToStations[10];
//vector<Berth *> g_stationsToGo[8]; // 物品类型对应去买卖的工作站
int g_recycleTypeCount[8];               // 当前可回收|物品类型的计数
//bitset<8> g_sellLock[MAX_WORK_STATION_SIZE];
//bitset<MAX_WORK_STATION_SIZE> g_buyLock;
int g_buyCount;
unordered_map<long long, vector<Point>> hash_paths; // 两点最短路径的Cache， TODO：将path对应的距离也进行存储

struct Robot;
vector<Robot> g_robots(ROBOT_NUM);
struct Good;
vector<Good> g_goods;

template<> struct std::hash<Point> {
    size_t operator()(const Point& p) const {
        // 使用了异或^和左移<<操作符来组合 “x和y坐标” 的哈希值
        return hash<int>()(p.x) ^ hash<int>()(p.y) << 1;
    }
};

inline double CalcDis(const Point &p1, const Point &p2)
{
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

inline int CalcPathDis(const vector<Point> &path)
{
    return path.size();
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


// 对两个坐标进行hash
long long hashTwoPoints(const Point& a, const Point& b) {
    const int shift = 16;    // 每8位存储一个值（使用16位以确保没有溢出）
    // 地图的大小是200，因此可以使用8位二进制存储（256）。
    // 通过位操作将[a.x, a.y, b.x, b.y]组合成一个long long类型的值，每个占8位
    return ((long long)a.x << (shift * 3)) | ((long long)a.y << (shift * 2)) |
           ((long long)b.x << (shift * 1)) | ((long long)b.y << (shift * 0));
}

// 检查坐标(x, y)是否在地图内以及是否可以走
bool isValid(int x, int y) {
    return x >= 0 && x < MAP_ARRAY_SIZE && y >= 0 && y < MAP_ARRAY_SIZE && (g_map[x][y] == '.'|| g_map[x][y] == 'A' || g_map[x][y] == 'B');
}

// bfs算法查找单源最短路径，结果存在hash_paths中(使用findPath来查找)
void calcPath(Point start, vector<Point>& endPoints) {
    queue<Point> q;
    q.push(start);
    unordered_map<Point, Point> prev; // 记录前驱节点，用于重建从起点到该终点的最短路径
    prev[start] = start; // 起点的前驱是自己
    vector<vector<bool>> visited(MAP_REAL_SIZE, vector<bool>(MAP_REAL_SIZE, false));
    visited[start.x][start.y] = true;

    while (!q.empty()) {
        Point cur = q.front();
        q.pop();

        // 检查当前点是否是任一终点
        for (auto& endPoint : endPoints) {
            long long hashKey = hashTwoPoints(start, endPoint);
            if (cur == endPoint) {
                if (hash_paths.find(hashKey) == hash_paths.end()) { // 如果路径未被记录
                    vector<Point> path;
                    for (Point p = cur; p != start; p = prev[p]) {
                        path.push_back(p);
                    }
                    path.push_back(start);
                    reverse(path.begin(), path.end());
                    hash_paths[hashKey] = path;
                }
                continue;
            }
        }

        // 遍历四个方向
        for (int i = 0; i < 4; ++i) {
            Point next(cur.x + DIRECTION_TO_GO[i][0], cur.y + DIRECTION_TO_GO[i][1]);
            if (isValid(next.x, next.y) && !visited[next.x][next.y]) {
                visited[next.x][next.y] = true;
                prev[next] = cur; // 记录到达next的前驱节点是cur
                q.push(next);
            }
        }
    }
}

// 从hash_paths中获取对应两点的最短路径
const vector<Point>& findPath(Point start, Point end) {
    static const vector<Point> emptyPath; // 静态空路径
    long long hashKey = hashTwoPoints(start, end);
    auto it = hash_paths.find(hashKey);
    if (it != hash_paths.end()) {
        return it->second; // 返回找到的路径的引用
    } else {
        return emptyPath; // 返回空路径的引用
    }
}

struct Good {
    Point p;                    // 货物坐标
    int value;                  // 货物金额(<= 1000)
    int restFrame;              // 剩余停留时间。 开始为1000帧。

    bool hasRobotLocked;        // 是否已有机器人前往

    Berth *targetBerth;         // 目标泊位
    int disToTargetBerth;       // 到目标泊位的距离
    vector<Point> pathToTargetBerth;    // 到最近泊位的路径，TODO:到N个泊位(Berth)的路径

    Good(int x, int y, int val)
    {
        p.x = x;
        p.y = y;
        value = val;

        restFrame = 1000;
        hasRobotLocked = false;
        findBerth();
    }
private:
    void findBerth()
    {
        calcPath(this->p, berths_point);
        for (auto berth: g_berths) {
            // TODO: 检查Berth
            auto path = findPath(this->p, berth.p);
            int dis = CalcPathDis(path);
            if (targetBerth == nullptr || dis < disToTargetBerth) {
                targetBerth = &berth;
                disToTargetBerth = dis;
                pathToTargetBerth = path;
            }
        }
    }
};

struct Robot
{
    int id;
    int nearWorkStation; // -1 表示没有靠近工作站
    Point p;             // 当前坐标
    int goods;          // 是否携带物品（0表示未携带物品，1表示携带物品）
    int status;         // 状态（0表示恢复状态，1表示正常运行状态）

    bool get;                   // 是否取货
    int disToTargetGood;        // 到目标货物的距离
    Good *targetGood;           // 目标货物
    bool pull;                  // 是否放置货物
    int disToTargetBerth;       // 到目标泊位的距离
    Berth *targetBerth;         // 目标泊位

    int nextStep;               // 路径中的下一个位置。 TODO：-1表示路径为空。
    vector<Point> path;         // 路径

    Robot() {}
    Robot(int startX, int startY) {
        p.x = startX;
        p.y = startY;
    }

    void Init()
    {
        get = false;
        pull = false;
        targetBerth = nullptr;
    }

    void findSuitableGood()
    {
        if (targetGood != nullptr) return;

        vector<Good> selectedGoods;     // 选择n个可运输的目标货物
        // TODO: 通过两点直线距离作为预估距离，选择top-n个最近的货物
        for (auto good: g_goods) {
            if (good.hasRobotLocked == false)
                selectedGoods.push_back(good);
        }
        vector<Point> targetPoints;
        for (auto good:selectedGoods) {
            targetPoints.push_back(good.p);
        }
        calcPath(this->p, targetPoints);    // 计算最短路径并存储hash path

        for (auto good: selectedGoods) {

        }


    }

private:
    int CalcMoveDirection(const Point &np) const
    {
        // 遍历四个方向
        for (int i = 0; i < 4; ++i) {
            if (p.x + DIRECTION_TO_GO[i][0] == np.x && p.y + DIRECTION_TO_GO[i][1] == np.y)
                return DIRECTION[i];
        }
        return -1;
    }
};

void Init()
{
    // 地图数据
    for(int i = 1; i <= MAP_REAL_SIZE; i ++)
        scanf("%s", g_map[i] + 1);
    // 泊位（Berth）数据
    for(int i = 0; i < BERTH_NUM; i ++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &g_berths[id].p.x, &g_berths[id].p.y, &g_berths[id].transport_time, &g_berths[id].loading_speed);
        berths_point.push_back(g_berths[id].p);     // 便于在创建Good时计算最近的Berth
    }
    // 船的容积
    scanf("%d", &g_boatCapacity);
    // 一行 OK
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}

void HandleFrame()
{
    for(int i = 0; i < ROBOT_NUM; i ++)
        printf("move %d %d\n", i, rand() % 4);
    return;
}

int main()
{
    ios::sync_with_stdio(false);
    cout.tie(nullptr);
    setbuf(stdout, nullptr);
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
//        int id = HandleFrame();
        // 帧序号、当前金钱数
        scanf("%d%d", &g_frameId, &g_money);
        int num;
        scanf("%d", &num);
        for(int i = 1; i <= num; i ++)  // 场上新增num个货物
        {
            int x, y;   // 新增货物坐标
            int val;    // 新增货物金额(<= 1000)
            scanf("%d%d%d", &x, &y, &val);
//            Good g(x, y, val);
//            g_goods.push_back(g);
        }
        for(int i = 0; i < ROBOT_NUM; i ++) // 机器人(Robot)
        {
            int sts;
            scanf("%d%d%d%d", &g_robots[i].goods, &g_robots[i].p.x, &g_robots[i].p.y, &sts);
        }
        for(int i = 0; i < 5; i ++)     // 船(Boat)
            scanf("%d%d\n", &g_boats[i].status, &g_boats[i].pos);
        scanf("%s", g_ok);
        printf("%d\n", g_frameId);
        HandleFrame();
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
