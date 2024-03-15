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
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

using namespace std;

ofstream logFile("log.txt", ios::trunc);            // 日志文件
ofstream logInfo("log_info.txt", ios::trunc);       // 调试
const double EPS = 1e-7;                            // 浮点数精度
const double FRAME_COUNT = 50;                      // 1s帧数
const int TOTAL_FRAME = 15000;                      // 总帧数
const int STOP_FRAME = 980;                         // 停留时间，20帧用于掉帧冗余
const int LIMIT_LOAD_FRAME = TOTAL_FRAME - 150;     // 终止装货的帧数

const int MAP_ARRAY_SIZE = 210;                     // 地图数组大小，预留一点空间
const int MAP_REAL_SIZE = 200;                      // 地图的真实大小
const int ROBOT_NUM = 10;                           // 机器人的数量
const int BOAT_NUM = 5;                             // 轮船的数量
const int BERTH_NUM = 10;                           // 泊位的数量
const int MAX_GOOD_NUM = MAP_REAL_SIZE * MAP_REAL_SIZE + 10;    // 货物的最大数量，预留点空间
const int TOP_K_SELECTED_BERTH_NUM = 2;             // 筛选距离最近的K个Berth
const int MAX_PATH_STEP = 4 * MAP_REAL_SIZE + MAP_REAL_SIZE;    // 最大的路程距离(绕地图一圈) + 倒退预留大小
const int MAX_RESET_PATH_STEP = 10;                 // 最大纠正步数
const double PREDICT_FRAME = 15;                    // 预测的帧数
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

int LOST_FRAME = 15000;                             // 丢失的帧数
int COUNT_VALUE = 0;                                // 泊位货物的总价值
struct Robot;
vector<Robot> g_robots(ROBOT_NUM);
char g_map[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
char g_ok[100];
int g_frameId;
int g_money;
int g_boatCapacity;
//vector<Point> berthsPullPoint(BERTH_NUM); // Init时将Berth的位置进行打包，便于在创建Good时计算最近的Berth
int berthsPullPoint[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];


struct Point;
struct Path;
bool CanHit(Point np);
void FixPos(Point &p);
bool IsValid(int x, int y);
long long HashTwoPoints(const Point &a, const Point &b);
long HashOnePoint(const Point &a);
void CalcPath(Point start, int (&endPoints)[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE]);
Path *FindPath(Point start, Point end);
bool getPath(Point p);


struct Point {
    int x;
    int y;

    Point(int x = -1, int y = -1) : x(x), y(y) {}

    void operator=(const Point &other) {
        x = other.x;
        y = other.y;
        return;
    }

    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point &other) const {
        return !(*this == other);
    }
};

Point Pre[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];

const Point INVALID_POINT = {-1, -1};       // 无效点

struct Path {
    Point path[MAX_PATH_STEP];  // 路径经过的点
    Path() : pathHead(MAP_REAL_SIZE), pathRear(MAP_REAL_SIZE), dis(1) {} // TODO：dis(1 )可能会导致BUG

    void printPath() {
        for (int i = MAP_REAL_SIZE; i < pathRear; ++i) {
            logFile << "(" << path[i].x << "," << path[i].y << ")";
        }
        logFile << endl;
    }

    bool checkCurrPoint(const Point &p) {
        logFile << " -------- check Curr Point --------" << p.x << " " << p.y << " pathHead:" << pathHead << " "
                << path[pathHead].x << " " << path[pathHead].y << endl;

        if (p != path[pathHead])printPath();

        if (pathHead == MAP_REAL_SIZE || p == path[pathHead])
            return true;
        else
            return false;
    }

    Point getNextPoint() {
        if (pathHead < pathRear) {
            pathHead++;
            //outFile << "getPoint" << pathHead << endl;
            return path[pathHead];
        } else {
            // 处理数组越界的情况
            std::cerr << "Path is too long, (" << pathHead << ")exceeding MAX_PATH_STEP." << std::endl;
            return INVALID_POINT;
        }
    }

    Point getBeforePoint() { // TODO
        pathHead = pathHead - 2;

        if (path[pathHead].x == -1) {
            bool flag = false;
            for (int i = 0; i < 4; i++) {
                Point tmp(path[pathHead + 1].x + DIRECTION_TO_GO[i][0], path[pathHead + 1].y + DIRECTION_TO_GO[i][1]);
                if (IsValid(tmp.x, tmp.y) && !CanHit(tmp)) {
                    flag = true;
                    path[pathHead].x = tmp.x;
                    path[pathHead].y = tmp.y;
                    break;
                }
            }
            logFile << " out of range " << " ========================================== " << endl;
            if (!flag)pathHead++;
        }
        return path[pathHead];
    }

    int getDis() {
        return dis;
    }

    // 添加路径点
    void addPoint(const Point &p) {
        if (pathRear < MAX_PATH_STEP) {
            //outFile << "addPoint" << pathRear << endl;
            path[pathRear++] = p;
            ++dis;
        } else {
            // 处理数组越界的情况
            std::cerr << "Path is too long, (" << pathRear << ")exceeding MAX_PATH_STEP." << std::endl;
        }
    }

    // 倒转路径点
    void reversePath() {

        //outFile << "pathHead（" << pathHead <<"）(pathRear - pathHead) / 2 + pathHead " << (pathRear - pathHead) / 2 + pathHead <<endl;
        for (int i = pathHead; i < (pathRear - pathHead) / 2 + pathHead; ++i) {
            // 交换元素，使用临时变量保存一个元素的值
            Point temp = path[i];
            path[i] = path[pathRear - 1 - i + pathHead];
            path[pathRear - 1 - i + pathHead] = temp;
        }
        //outFile <<" afterReverse" << endl;

    }

    void resetHead() {
        this->pathHead = 200;
        int tmp = pathHead - 1;
        while (path[tmp].x != -1) {
            path[tmp].x = -1;
            tmp--;
        }
    }

    Point finallyPonit() {
        return path[pathRear - 1];
    }

    int pathHead;               // path头
    int pathRear;               // path尾
    int dis;                    // 路径的总距离


};

template<>
struct std::hash<Point> {
    size_t operator()(const Point &p) const {
        // 使用了异或^和左移<<操作符来组合 “x和y坐标” 的哈希值
        return hash<int>()(p.x) ^ hash<int>()(p.y) << 1;
    }
};

unordered_map<long long, Path> hash_paths; // 两点最短路径的Cache
unordered_map<Point, Path> back_paths; // 两点最短路径的Cache

struct Berth {
    int id;
    Point p;
    Point pullP;
    int transportTime;  // 表示该泊位轮船运输到虚拟点的时间
    int loadingSpeed;   // 表示该泊位的装载速度，即每帧可以装载的物品数，单位(个)
    int stackGoodNum;   // 堆积的货物数量
    bool hasBoatLocked; // 是否有船占用

    Berth() {
        hasBoatLocked = false;
    }

    void Init() {
        this->stackGoodNum = 0;
        FixPos(this->p);  // 得到的位置是从0开始的，+1与地图保持一致
        CalcPullPoint();
    }

private:
    Point CalcPullPoint() {
        this->pullP.x = this->p.x;
        this->pullP.y = this->p.y;
        return this->pullP;
    }
};

Berth g_berths[BERTH_NUM];

struct Boat {
    int id;
    int capacity;
    int berthId;        // 表示目标泊位
    int status;     // 状态(0表示运输中；1表示正常运行状态，即装货中或运输完成；2表示泊位外等待)
    int finishTransportFrame; // 运输结束时间

    // 船的状态
    // 运输状态（status == 0）：不管它
    //
    Boat() {
        finishTransportFrame = 0;
    }

    int FindSuitableBerth() {
//        logBoat << "------Function: FindSuitableBerth()" << endl;
        Berth *tmpBerth;
        int tmpStackGoodNum = 0;
        this->berthId = -1;
        for (int i = 0; i < BERTH_NUM; i++) {
//            logBoat << "berth(" << i << "): " <<
//                    "!hasBoatLocked(" << !g_berths[i].hasBoatLocked << ") " <<
//                    "stackGoodNum(" << g_berths[i].stackGoodNum << ")" << endl;
            if (!g_berths[i].hasBoatLocked && g_berths[i].stackGoodNum > tmpStackGoodNum) {
                tmpBerth = &g_berths[i];
                this->berthId = tmpBerth->id;
//                logBoat << "Found berth(" << i << ")" << tmpBerth->id << endl;
            }
        }
        if (tmpBerth != nullptr) {
            tmpBerth->hasBoatLocked = true;
            return tmpBerth->id;
        }

//        logBoat << "------Function: FindSuitableBerth() end" << endl;
        return -1;
    }
};

Boat g_boats[BOAT_NUM];

struct Good {
    Point p;                    // 货物坐标
    int value;                  // 货物金额(<= 1000)
    int restFrame;              // 剩余停留时间。 开始为1000帧。
    int startFrame;             // 出现时间
    bool hasRobotLocked;        // 是否已有机器人前往
    bool canShip;               // 是否可以运送

    Berth *targetBerth;         // 目标泊位
    int disToTargetBerth;       // 到目标泊位的距离
    Path *pathToTargetBerth;    // 到目标泊位的路径

    Good() {}

    void Init(int startFrame) {
        targetBerth = nullptr;
        disToTargetBerth = 40000;
        pathToTargetBerth = nullptr;

        FixPos(this->p);  // 得到的位置是从0开始的，+1与地图保持一致
        restFrame = 1000;
        hasRobotLocked = false;
        canShip = true;
        this->startFrame = startFrame;
    }

    void findBerth() {
        if (!getPath(this->p)) {
            this->canShip = false;
            return;
        }
//        for (auto g_berth: g_berths) {
        // TODO: 检查Berth
        auto it = back_paths.find(this->p);
        auto path = &(it->second);
        int dis = path->getDis();

        logFile << "  Point " << path->finallyPonit().x << " " << path->finallyPonit().y << endl;
        //path->printPath();


        for (auto &g_berth: g_berths) {
            if (path->finallyPonit() == g_berth.p) {
                this->targetBerth = &g_berth;
            }
        }

        this->disToTargetBerth = dis;
        this->pathToTargetBerth = path;

    }
};

typedef struct GoodNode {
    Good good;
    struct GoodNode *next;
    struct GoodNode *prev;  // 前一个节点，便于删除
} GoodNode;

struct GoodList {   // 货物清单
    GoodNode *head;

    GoodList() {
        this->head = (GoodNode *) malloc(sizeof(GoodNode));    // 头节点
        this->head->next = this->head;
        this->head->prev = this->head;
        this->length = 0;
    }

    // 判断货物清单是否为空
    bool isEmpty() {
        return this->head->next == this->head;
    }

    int getLength() {
        return this->length;
    }

    // 增加一个货物
    void addGood(Good item) {
        GoodNode *newNode = (GoodNode *) malloc(sizeof(GoodNode));
        newNode->good = item;
        newNode->next = this->head;         // 新节点的下一个是头节点(插在队尾)
        newNode->prev = this->head->prev; // 新节点的前一个是当前的队尾

        this->head->prev->next = newNode; // 当前队尾的下一个是新节点
        this->head->prev = newNode;         // 头节点的前一个现在是新节点
        this->length++;
    }

    // 删除一个货物
    void deleteGood(GoodNode *item) {
        if (item == nullptr || item == this->head) {
            return;
        }
        item->prev->next = item->next;
        item->next->prev = item->prev;

        this->length--;
        free(item);
    }

    void traverseGoodList(GoodList *list) {
        if (list->isEmpty()) {
            return;
        }
        // For
        for (GoodNode *curr = list->head->next; curr != list->head; curr = curr->next) {
            printf("Good: %d\n", curr->good.restFrame);
        }
        // While
        GoodNode *curr = list->head->next;
        while (curr != list->head) {
            printf("Good: %d\n", curr->good.restFrame);
            curr = curr->next;
        }
    }

    void deleteTimeOut(int frame) {
        GoodNode *cur = head->next;
        while (cur->good.startFrame + STOP_FRAME < frame) {
            GoodNode *tmp = cur;
            cur = cur->next;
            deleteGood(tmp);
        }
    }


private:
    int length;
} g_goodList;

struct Robot {
    int id;
    int nearWorkStation; // -1 表示没有靠近工作站
    Point p;             // 当前坐标
    int goods;          // 是否携带物品（0表示未携带物品，1表示携带物品）
    int status;         // 状态（0表示恢复状态，1表示正常运行状态）
    int value = 0;          // 携带的货物价值
    Path *path;                  // 路径
    int move;                   // 移动方向

    // 目标货物
    void setTargetGood(Good &good) {
        this->targetGood = &good;
        this->value = this->targetGood->value;      // 记录货物价值
        this->targetBerth = nullptr;
    }

    Good *getTargetGood() {
        return this->targetGood;
    }

    bool get;                   // 是否取货

    // 目标泊位
    void setTargetBerth(Good *good) {
        this->targetBerth = good->targetBerth;
        this->path->resetHead();
        this->path = good->pathToTargetBerth;
        this->path->printPath();
        this->path->resetHead();
        this->targetGood = nullptr;
    }

    Berth *getTargetBerth() {
        return this->targetBerth;
    }

    bool pull;                  // 是否放置货物


    // 恢复状态：status == 0
    // 正常状态但没有分配任务：status == 1, goods == 0, targetGood == nullptr, targetBerth == nullptr
    // 正常状态且正在前往取货：status == 1, goods == 0, targetGood != nullptr
    // 正常状态且正在送货：status == 1, goods == 1, targetBerth != nullptr
    Robot() {
        path = nullptr;
        targetGood = nullptr;
        targetBerth = nullptr;
        goods = 0;
        status = 1;
        value = 0;
    }

    void resetStatus() {
        this->targetBerth = nullptr;
        this->targetGood = nullptr;
        this->path = nullptr;
        this->value = 0;    // 清除携带货物价值
    }


    void CheckStatus() {
        FixPos(this->p);  // 得到的位置是从0开始的，+1与地图保持一致
        this->move = -1;              // -1表示不对机器人下达指令
        this->get = false;
        this->pull = false;
        if (status == 0) return;
        // 当前处在“正常状态且正在送货”, 当前点处在泊位，货物已送出：重置机器人状态为“正常状态但没有分配任务”
        if (this->targetBerth != nullptr && this->goods == 0 && InBerth(this->targetBerth->p ,this->p)) {
            this->resetStatus();
        }

        // 错误状态   取货超时
        if (this->targetBerth != nullptr && this->goods == 0 && this->p != this->targetBerth->pullP) {
            this->resetStatus();
        }

        // “正常状态且正在前往取货”, 当前点处在货物位置，并取到货物：重置机器人状态为“正常状态且正在送货”
        if (this->targetGood != nullptr && this->goods == 1 && this->p == this->targetGood->p) {
            this->targetBerth = this->targetGood->targetBerth;
            this->path->resetHead();

            //getPath(p);
            this->path = this->targetGood->pathToTargetBerth;


            this->path->resetHead();
            this->targetGood = nullptr;
        }
    }

    bool InBerth(Point target, Point next) {
        if (next.x >= target.x && next.x <= target.x + 3 && next.y >= target.y && next.y <= target.y + 3)return true;
        return false;
    }

    void CalcNextStep() {

        // TODO: 矫正path step
//        int reset_step = -1;
//        for (int i = 1; i <= MAX_RESET_PATH_STEP; ++i) {
//            auto currPoint = path[nextStep-i];
//            if (currPoint == this->p) {
//                break;
//            }
//        }

        this->get = false;
        this->pull = false;

        //if(this->path != nullptr)outFile << " checkCurr" << " " << this->path->checkCurrPoint(p) <<endl;

        //&& this->path->checkCurrPoint(p)F

        if (this->path == nullptr) {
            move = -1;
            return;
        }
        if (this->path->checkCurrPoint(p) == false) {
            this->targetBerth = nullptr;
            this->targetGood = nullptr;
            this->path = nullptr;
        } else {
            auto nextPoint = path->getNextPoint();

            if (CanHit(nextPoint)) {

                nextPoint = path->getBeforePoint();

                //TO ADD
                if(nextPoint == p){
                    move = -1;
                    return;
                }

                if (CanHit(nextPoint)) {
                    move = -1;
                    path->pathHead = path->pathHead + 1;
                    return;
                } else {
                    move = CalcMoveDirection(nextPoint);
                    this->p.x = nextPoint.x;
                    this->p.y = nextPoint.y;
                    logFile << " ============ " << this->p.x << " " << this->p.y << endl;
                    return;
                }
            }


            move = CalcMoveDirection(nextPoint);



            // 检查是否走到最后一步的前一步，更新get或pull指令
            if (this->targetGood != nullptr && this->targetGood->p == nextPoint) {
                this->get = true;
                this->targetBerth = this->targetGood->targetBerth;
            }

            //outGetPull << "robot:" << id << " " <<nextPoint.x <<" "<<nextPoint.y<<(this->targetBerth != nullptr) << endl;

            if (this->targetBerth != nullptr)
                logFile << " TTTTTT " << this->targetBerth->p.x << " " << this->targetBerth->p.y << " " << nextPoint.x
                        << " " << nextPoint.y << endl;

            if (this->targetBerth != nullptr && InBerth(this->targetBerth->p ,nextPoint) && this->goods == 1) {
                this->pull = true;
                this->targetBerth->stackGoodNum += 1;
            }

            this->p.x = nextPoint.x;
            this->p.y = nextPoint.y;

            return;

        }

    }

    void FindSuitableGood(int frame) {
        if (targetGood != nullptr) return;

        //vector<Point> goodsPoint(g_goodList.getLength());     // 选择n个可运输的目标货物
        // TODO: 通过两点直线距离作为预估距离，选择top-n个最近的货物
        int i = 0;

        int goodsPoint[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
        memset(goodsPoint, -2, sizeof(goodsPoint));


        for (GoodNode *curr = g_goodList.head->next; curr != g_goodList.head; curr = curr->next) {
            if (!curr->good.hasRobotLocked && curr->good.canShip) {
                goodsPoint[curr->good.p.x][curr->good.p.y] = -1;
            }
        }
        // 计算 货物价值 / (机器人到货物的距离 + 货物到泊位的距离)，按顺序检查是否可以在货物消失前搬运

        CalcPath(this->p, goodsPoint);
        Good *tempTargetGood;
        Path *tempPathToGood;
        double vpdToTargetBerth = 0.0;   // Value per dis
        Good *tmp = nullptr;
        for (GoodNode *curr = g_goodList.head->next; curr != g_goodList.head; curr = curr->next) {
            if (!curr->good.hasRobotLocked) {

                auto pathToGood = FindPath(this->p, curr->good.p);
                if (pathToGood == nullptr) continue;    // 货物不可达
                int dis = pathToGood->getDis();
                double valuePerDis = curr->good.value / (curr->good.pathToTargetBerth->getDis() + dis);
                if (targetGood == nullptr ||
                    (valuePerDis > vpdToTargetBerth && frame + dis < curr->good.startFrame + STOP_FRAME)) {
                    logFile << " --point-- " << curr->good.p.x << " " << curr->good.p.y << endl;
                    vpdToTargetBerth = valuePerDis;
                    this->targetGood = &curr->good;
                    this->value = curr->good.value;
                    //this->targetGood->hasRobotLocked = true; // 锁定货物
                    tmp = this->targetGood;
                    this->path = pathToGood;
                    this->value = this->targetGood->value;  // 记录货物的价值
                }
            }
        }

        if (tmp != nullptr)
            tmp->hasRobotLocked = true;


        //outFile << "----------------" << (this->targetGood != nullptr) << " " << vpdToTargetBerth << endl;
    }


private:
    Good *targetGood;           // 目标货物
    Berth *targetBerth;         // 目标泊位
    int CalcMoveDirection(const Point &np) const {
        // 遍历四个方向
        for (int i = 0; i < 4; ++i) {
            if (p.x + DIRECTION_TO_GO[i][0] == np.x && p.y + DIRECTION_TO_GO[i][1] == np.y)
                return DIRECTION[i];
        }
        return -1;
    }
};



bool CanHit(Point np) {
    for (int i = 0; i < ROBOT_NUM; i++) {
        if (np.x == g_robots[i].p.x && np.y == g_robots[i].p.y) {
            return true;
        }
    }
    return false;
}

// 修复点
void FixPos(Point &p) {
    p.x += 1;
    p.y += 1;
}

inline double CalcDis(const Point &p1, const Point &p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

inline bool IsEq(double x1, double x2, const double eps = EPS) {
    return abs(x1 - x2) < eps;
}

// 检查坐标(x, y)是否在地图内以及是否可以走
bool IsValid(int x, int y) {
    return x >= 0 && x < MAP_ARRAY_SIZE && y >= 0 && y < MAP_ARRAY_SIZE &&
           (g_map[x][y] == '.' || g_map[x][y] == 'A' || g_map[x][y] == 'B');
}

// 对两个坐标点进行hash
long long HashTwoPoints(const Point &a, const Point &b) {
    const int shift = 16;    // 每8位存储一个值（使用16位以确保没有溢出）
    // 地图的大小是200，因此可以使用8位二进制存储（256）。
    // 通过位操作将[a.x, a.y, b.x, b.y]组合成一个long long类型的值，每个占8位
    return ((long long) a.x << (shift * 3)) | ((long long) a.y << (shift * 2)) |
           ((long long) b.x << (shift * 1)) | ((long long) b.y << (shift * 0));
}

// 对一个坐标点进行hash
long HashOnePoint(const Point &a) {
    const int shift = 16;    // 每8位存储一个值（使用16位以确保没有溢出）
    // 地图的大小是200，因此可以使用8位二进制存储（256）。
    // 通过位操作将[a.x, a.y, b.x, b.y]组合成一个long long类型的值，每个占8位
    return ((long long) a.x << (shift * 1)) | ((long long) a.y << (shift * 0));
}

// bfs算法查找单源最短路径，结果存在hash_paths中(使用findPath来查找)
void CalcPath(Point start, int (&endPoints)[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE]) {
    queue<Point> q;
    q.push(start);
    Point prev[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];   // 记录前驱节点，用于重建从起点到该终点的最短路径
    // 初始化prev数组
    for (int i = 0; i < MAP_REAL_SIZE; i++) {
        for (int j = 0; j < MAP_REAL_SIZE; j++) {
            prev[i][j] = INVALID_POINT;
        }
    }
    prev[start.x][start.y] = start; // 起点的前驱是自己
    bool visited[MAP_REAL_SIZE][MAP_REAL_SIZE] = {};
    memset(visited, false, sizeof(visited));
    visited[start.x][start.y] = true;

    while (!q.empty()) {
        Point cur = q.front();
        q.pop();

        //outFile<<"--- CalcPath ---" << cur.x << " " << cur.y << endl;
        if (endPoints[cur.x][cur.y] == -1) {
            Point endPoint(cur.x, cur.y);

            if(endPoint == start)continue;
            long long hashKey = HashTwoPoints(start, endPoint);
            //outFile<<"--- CalcPath end---" << cur.x << " " << cur.y <<" "<<(hash_paths.find(hashKey) == hash_paths.end())<< endl;
            if (hash_paths.find(hashKey) == hash_paths.end()) { // 如果路径未被记录
                Path path;

                for (Point p = cur; p != start; p = prev[p.x][p.y]) {
                    path.addPoint(p);
                }

                path.addPoint(start);


                path.reversePath();
                hash_paths[hashKey] = path;

            }
            continue;
        }



        // 遍历四个方向
        for (int i = 0; i < 4; ++i) {
            Point next(cur.x + DIRECTION_TO_GO[i][0], cur.y + DIRECTION_TO_GO[i][1]);
            if (IsValid(next.x, next.y) && !visited[next.x][next.y]) {
                visited[next.x][next.y] = true;
                prev[next.x][next.y] = cur; // 记录到达next的前驱节点是cur
                q.push(next);
            }
        }
    }
}

// 从hash_paths中获取对应两点的最短路径
Path *FindPath(Point start, Point end) {
    long long hashKey = HashTwoPoints(start, end);
    auto it = hash_paths.find(hashKey);
    if (it != hash_paths.end()) {
        return &it->second; // 返回找到的路径的引用
    } else {


        return nullptr;
    }
}

// 查找货物到最近泊位的路径
bool getPath(Point p) {
    if (Pre[p.x][p.y] == INVALID_POINT) {
        return false;
    }
    if (back_paths.find(p) == back_paths.end()) { // 如果路径未被记录
        Path path;
        Point cur = p;
        while (cur != Pre[cur.x][cur.y]) {
            path.addPoint(cur);
            cur = Pre[cur.x][cur.y];
        }
        path.addPoint(cur);
        back_paths[p] = path;
        return true;
    }
    return true;
}


// Game control logic

void HandleFrame(int frame) {
    //删除超时节点
    g_goodList.deleteTimeOut(frame - 500);

    // 为新增的每个货物找到最近的泊位（避免重复计算）
    for (GoodNode *curr = g_goodList.head->next; curr != g_goodList.head; curr = curr->next) {
        if (curr->good.canShip && curr->good.targetBerth == nullptr) {
            curr->good.findBerth();
//            outGetPull << "123currGoodTargetBerth:" << (curr->good.targetBerth == nullptr)<<endl;
        }
    }

    // 此处策略：如果机器人当前有工作：计算机器人下一步动作
    for (int i = 0; i < ROBOT_NUM; i++) {

        //outFile << "robot before cal:+++++++++++++++++" << i <<" " << g_robots[i].p.x <<" " << g_robots[i].p.y << endl;

        logFile << "------------status robot" << i << " :" << g_robots[i].status << " " << g_robots[i].goods << " "
                << (g_robots[i].getTargetGood() != nullptr) << " " << (g_robots[i].getTargetBerth() != nullptr) << endl;

        // 对机器人的四个状态进行处理
        if (g_robots[i].status == 0) continue;  // 恢复状态
        if (g_robots[i].goods == 0 && g_robots[i].getTargetGood() == nullptr &&
            g_robots[i].getTargetBerth() == nullptr) {     // 正常状态但没有分配任务
            g_robots[i].FindSuitableGood(frame);
            g_robots[i].CalcNextStep();
        } else if (g_robots[i].goods == 0 && g_robots[i].getTargetGood() != nullptr) { // 正常状态且正在前往取货
            g_robots[i].CalcNextStep();
        } else if (g_robots[i].goods == 1 && g_robots[i].getTargetBerth() != nullptr) {  // 正常状态且正在送货
            g_robots[i].CalcNextStep();
        } else {    // 未确认的状态
            logFile << "Robot[" << i << "]: goods(" << g_robots[i].goods <<
                    "), TargetGood!=NULL(" << (g_robots[i].getTargetGood() != nullptr) <<
                    "), TargetBerth!=NULL(" << (g_robots[i].getTargetBerth() != nullptr) <<
                    ")" << endl;
            continue;
        }

        //outFile << "robot after cal:+++++++++++++++++++" << i <<" " << g_robots[i].p.x <<" " << g_robots[i].p.y << endl;

        // 输出机器人控制指令

        logFile << " after action: " << g_robots[i].move << " " << g_robots[i].get << " " << g_robots[i].pull << endl;

        if (g_robots[i].move != -1)
            printf("move %d %d\n", i, g_robots[i].move);

        if (g_robots[i].get) {
            printf("get %d\n", i);
        } else if (g_robots[i].pull) {
            //
            printf("pull %d\n", i);
            COUNT_VALUE += g_robots[i].value;
        }
    }

    // 此处策略：
    for (int i = 0; i < BOAT_NUM; i++) {    // 船(Boat)
        if (frame + g_berths[g_boats[i].berthId].transportTime > LIMIT_LOAD_FRAME) {    // 没时间
            g_berths[g_boats[i].berthId].hasBoatLocked = false;
            printf("go %d\n", i);
            g_boats[i].finishTransportFrame = frame + g_berths[g_boats[i].berthId].transportTime;
        }

        if (g_boats[i].status == 1 && g_boats[i].berthId == -1) {   // 起始状态
            g_boats[i].FindSuitableBerth();
            if (g_boats[i].berthId != -1) {
                printf("ship %d %d\n", i, g_boats[i].berthId);
                g_boats[i].capacity = 0;
            }
            continue;
        }
//        if (g_boats[i].status == 0 && frame > g_boats[i].finishTransportFrame) {
//        }
//        else
        if (g_boats[i].status == 0) {
            if (frame >= g_boats[i].finishTransportFrame) {
                g_boats[i].FindSuitableBerth();
                if (g_boats[i].berthId != -1) {
                    printf("ship %d %d\n", i, g_boats[i].berthId);
                    g_boats[i].capacity = 0;
                    g_boats[i].finishTransportFrame = frame + g_berths[g_boats[i].berthId].transportTime;
                }
            } else if (frame < g_boats[i].finishTransportFrame)
                continue;
        } else if (g_boats[i].status == 1) {
            if (g_berths[g_boats[i].berthId].stackGoodNum == 0) {  // 泊位没货了
                g_berths[g_boats[i].berthId].hasBoatLocked = false;
                g_boats[i].FindSuitableBerth();
                if (g_boats[i].berthId != -1) {
                    g_berths[g_boats[i].berthId].hasBoatLocked = true;
                    printf("ship %d %d\n", i, g_boats[i].berthId);
                }
            } else if (g_boats[i].capacity != 100 && g_berths[g_boats[i].berthId].stackGoodNum != 0) {
                g_boats[i].capacity += 1;
                g_berths[g_boats[i].berthId].stackGoodNum -= 1;
            } else if (g_boats[i].capacity == 100) {    // 装满
                g_berths[g_boats[i].berthId].hasBoatLocked = false;
                printf("go %d\n", i);
                g_boats[i].finishTransportFrame = frame + g_berths[g_boats[i].berthId].transportTime;
            }
        }
    }

    LOST_FRAME--;

    logFile << " ========= Lost =====" << LOST_FRAME << " " << COUNT_VALUE << endl;
//    for (GoodNode *curr = g_goodList.head->next; curr != g_goodList.head; curr = curr->next) {
//        if (!curr->good.hasRobotLocked) {
//            --curr->good.restFrame;       // 货物剩余时间-1
//        }
//    }
    return;
}

void InitPre() {
    queue<Point> q;
    for (int i = 0; i < MAP_REAL_SIZE; i++) {
        for (int j = 0; j < MAP_REAL_SIZE; j++) {
            Pre[i][j] = INVALID_POINT;
        }
    }
    bool visited[MAP_ARRAY_SIZE][MAP_ARRAY_SIZE];
    memset(visited, false, sizeof(visited));
    for (int i = 0; i < BERTH_NUM; i++) {
        Point start(g_berths[i].p.x, g_berths[i].p.y);
        q.push((start));
        visited[start.x][start.y] = true;
        Pre[start.x][start.y] = start;
        logFile << " init pre" << start.x << " " << start.y << endl;
    }

    while (!q.empty()) {
        Point cur = q.front();
        q.pop();
        for (int i = 0; i < 4; ++i) {
            Point next(cur.x + DIRECTION_TO_GO[i][0], cur.y + DIRECTION_TO_GO[i][1]);
            if (IsValid(next.x, next.y) && !(visited[next.x][next.y])) {
                visited[next.x][next.y] = true;
                Pre[next.x][next.y] = cur; // 记录到达next的前驱节点是cur
                q.push(next);
            }
        }
    }


//    for (int i = 0; i < MAP_REAL_SIZE; i++) {
//        for (int j = 0; j < MAP_REAL_SIZE; j++) {
//            outFile << setw(4) << Pre[i][j].x << " " <<setw(4)<< Pre[i][j].y <<" | ";
//        }
//        outFile << endl;
//    }
}

void Init() {
    // 地图数据
    logFile << "----------------init---------------" << endl;
    for (int i = 1; i <= MAP_REAL_SIZE; i++)
        scanf("%s", g_map[i] + 1);
    // 泊位（Berth）数据
    logFile << "----Berth" << endl;
    memset(berthsPullPoint, -2, sizeof(berthsPullPoint));
    for (int i = 0; i < BERTH_NUM; i++) {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &g_berths[id].p.x, &g_berths[id].p.y, &g_berths[id].transportTime,
              &g_berths[id].loadingSpeed);
        g_berths[id].id = id;   // 存储id
        g_berths[id].Init();    // 初始化
        berthsPullPoint[g_berths[id].pullP.x][g_berths[id].pullP.y] = -1;
        logFile << g_berths[id].p.x << " " << g_berths[id].p.y << endl;
    }
    // 船的容积
    scanf("%d", &g_boatCapacity);


    InitPre();


    // 一行 OK
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);


}

int main() {
    ios::sync_with_stdio(false);
    cout.tie(nullptr);
    setbuf(stdout, nullptr);
    Init();
    for (int frame = 1; frame <= TOTAL_FRAME; frame++) {
        // 帧序号、当前金钱数
        scanf("%d%d", &g_frameId, &g_money);

        logFile << g_frameId << " --------------------------------------------------------- " << endl;

        int num;
        scanf("%d", &num);

        logFile << "----新增货物数量：" << num << endl;

        for (int i = 1; i <= num; i++)  // 场上新增num个货物
        {
            Good g;
            scanf("%d%d%d", &g.p.x, &g.p.y, &g.value);

            logFile << "--------" << i << "  :" << setw(3) << g.p.x << " " << setw(3) << g.p.y << " " << g.value
                    << endl;

            g.Init(frame);    // 初始化
            g_goodList.addGood(g);
        }

        logFile << "----robot" << endl;

        for (int i = 0; i < ROBOT_NUM; i++) {   // 机器人(Robot)
            g_robots[i].id = i;
            scanf("%d%d%d%d", &g_robots[i].goods, &g_robots[i].p.x, &g_robots[i].p.y, &g_robots[i].status);

            g_robots[i].CheckStatus();

            logFile << "--------" << i << "  :" << setw(3) << g_robots[i].goods << " " << setw(3) << g_robots[i].p.x
                    << " " << setw(3) << g_robots[i].p.y << " " << g_robots[i].status << endl;
        }

        logFile << "----boat" << endl;

        for (int i = 0; i < BOAT_NUM; i++) {    // 船(Boat)
            g_boats[i].id = i;

            //outFile << "--------" << i << " :" << g_robots[i].status << " " << g_boats[i].berthId << endl;
            logFile << "--------" << i << "  : berth(" << g_boats[i].berthId << ") status(" << g_boats[i].status << ")"
                    << endl;

            scanf("%d%d\n", &g_boats[i].status, &g_boats[i].berthId);
        }

        scanf("%s", g_ok);
        printf("%d\n", g_frameId);
        HandleFrame(frame);
        puts("OK");
        fflush(stdout);
    }


    logFile.close();
    return 0;
}